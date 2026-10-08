/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <karto_sdk/Types.h>
#include <math.h>
#include <assert.h>
#include <boost/serialization/vector.hpp>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <queue>
#include <set>
#include <list>
#include <iterator>
#include <map>
#include <vector>
#include <utility>
#include <algorithm>
#include <string>

#include "karto_sdk/Mapper.h"

#include "Eigen/Eigenvalues"

// Must live at global scope (not inside `namespace karto`): BOOST_CLASS_VERSION expands to
// `namespace boost { namespace serialization { ... } }`, and nesting that inside `namespace
// karto` would define a bogus karto::boost::serialization instead of ::boost::serialization,
// silently shadowing it for every `friend class boost::serialization::access;` declared later
// in this translation unit.
// v1 added sensor-frame data, v2 added odometry-frame data (both for online joint
// lidar<->base_link extrinsic calibration -- see PLAN.md and its addendum).
BOOST_CLASS_VERSION(karto::LinkInfo, 2)

BOOST_CLASS_EXPORT(karto::MapperGraph);
BOOST_CLASS_EXPORT(karto::Graph<karto::LocalizedRangeScan>);
BOOST_CLASS_EXPORT(karto::EdgeLabel);
BOOST_CLASS_EXPORT(karto::LinkInfo);
BOOST_CLASS_EXPORT(karto::Edge<karto::LocalizedRangeScan>);
BOOST_CLASS_EXPORT(karto::Vertex<karto::LocalizedRangeScan>);
BOOST_CLASS_EXPORT(karto::MapperSensorManager)
BOOST_CLASS_EXPORT(karto::Mapper)
namespace karto
{

// enable this for verbose debug information
// #define KARTO_DEBUG

  #define MAX_VARIANCE            500.0
  #define DISTANCE_PENALTY_GAIN   0.2
  #define ANGLE_PENALTY_GAIN      0.2

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Manages the scan data for a device
 */
class ScanManager
{
public:
  /**
   * Default constructor
   */
  ScanManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
  : m_pLastScan(NULL),
    m_RunningBufferMaximumSize(runningBufferMaximumSize),
    m_RunningBufferMaximumDistance(runningBufferMaximumDistance),
    m_NextStateId(0)
  {
  }

  ScanManager() {}

  /**
   * Destructor
   */
  virtual ~ScanManager()
  {
    Clear();
  }

public:
  /**
   * Adds scan to vector of processed scans tagging scan with given unique id
   * @param pScan
   */
  inline void AddScan(LocalizedRangeScan * pScan, kt_int32s uniqueId)
  {
    // assign state id to scan
    pScan->SetStateId(m_NextStateId);

    // assign unique id to scan
    pScan->SetUniqueId(uniqueId);

    // add it to scan buffer
    m_Scans.insert({pScan->GetStateId(), pScan});
    m_NextStateId++;
  }

  /**
   * Gets last scan
   * @param deviceId
   * @return last localized range scan
   */
  inline LocalizedRangeScan * GetLastScan()
  {
    return m_pLastScan;
  }

  /**
   * Clears last scan
   * @param deviceId
   */
  inline void ClearLastScan()
  {
    m_pLastScan = NULL;
  }

  /**
   * Sets the last scan
   * @param pScan
   */
  void SetLastScan(LocalizedRangeScan * pScan)
  {
    m_pLastScan = pScan;
  }

  /**
   * Gets scans
   * @return scans
   */
  inline LocalizedRangeScanMap & GetScans()
  {
    return m_Scans;
  }

  /**
   * Gets running scans
   * @return running scans
   */
  inline LocalizedRangeScanVector & GetRunningScans()
  {
    return m_RunningScans;
  }

  /**
   * Gets running scan buffer size
   * @return running scan buffer size
   */
  inline kt_int32u & GetRunningScanBufferSize()
  {
    return m_RunningBufferMaximumSize;
  }

  /**
   * Sets running scan buffer size
   * @param rScanBufferSize
   */
  void SetRunningScanBufferSize(const kt_int32u & rScanBufferSize)
  {
    m_RunningBufferMaximumSize = rScanBufferSize;
  }

  /**
   * Sets running scan buffer maximum distance
   * @param rScanBufferMaxDistance
   */
  void SetRunningScanBufferMaximumDistance(const kt_int32u & rScanBufferMaxDistance)
  {
    m_RunningBufferMaximumDistance = rScanBufferMaxDistance;
  }

  /**
   * Adds scan to vector of running scans
   * @param pScan
   */
  void AddRunningScan(LocalizedRangeScan * pScan)
  {
    m_RunningScans.push_back(pScan);

    // vector has at least one element (first line of this function), so this is valid
    Pose2 frontScanPose = m_RunningScans.front()->GetSensorPose();
    Pose2 backScanPose = m_RunningScans.back()->GetSensorPose();

    // cap vector size and remove all scans from front of vector that are too far from end of vector
    kt_double squaredDistance = frontScanPose.GetPosition().SquaredDistance(
      backScanPose.GetPosition());
    while (m_RunningScans.size() > m_RunningBufferMaximumSize ||
      squaredDistance > math::Square(m_RunningBufferMaximumDistance) - KT_TOLERANCE)
    {
      // remove front of running scans
      m_RunningScans.erase(m_RunningScans.begin());

      // recompute stats of running scans
      frontScanPose = m_RunningScans.front()->GetSensorPose();
      backScanPose = m_RunningScans.back()->GetSensorPose();
      squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
    }
  }

  /**
   * Finds and replaces a scan from m_scans with NULL
   * @param pScan
   */
  void RemoveScan(LocalizedRangeScan * pScan)
  {
    LocalizedRangeScanMap::iterator it = m_Scans.find(pScan->GetStateId());
    if (it != m_Scans.end()) {
      it->second = NULL;
      m_Scans.erase(it);
    } else {
      std::cout << "Remove Scan: Failed to find scan in m_Scans" << std::endl;
    }
  }

  /**
   * Clears the vector of running scans
   */
  void ClearRunningScans()
  {
    m_RunningScans.clear();
  }

  /**
   * Deletes data of this buffered device
   */
  void Clear()
  {
    m_Scans.clear();
    m_RunningScans.clear();
  }

private:
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Scans);
    ar & BOOST_SERIALIZATION_NVP(m_RunningScans);
    ar & BOOST_SERIALIZATION_NVP(m_pLastScan);
    ar & BOOST_SERIALIZATION_NVP(m_RunningBufferMaximumSize);
    ar & BOOST_SERIALIZATION_NVP(m_RunningBufferMaximumDistance);
    ar & BOOST_SERIALIZATION_NVP(m_NextStateId);
  }

private:
  LocalizedRangeScanMap m_Scans;
  LocalizedRangeScanVector m_RunningScans;
  LocalizedRangeScan * m_pLastScan;
  kt_int32u m_NextStateId;

  kt_int32u m_RunningBufferMaximumSize;
  kt_double m_RunningBufferMaximumDistance;
};    // ScanManager

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void MapperSensorManager::RegisterSensor(const Name & rSensorName)
{
  if (GetScanManager(rSensorName) == NULL) {
    m_ScanManagers[rSensorName] = new ScanManager(
      m_RunningBufferMaximumSize,
      m_RunningBufferMaximumDistance);
  }
}


/**
 * Gets scan from given device with given ID
 * @param rSensorName
 * @param scanNum
 * @return localized range scan
 */
LocalizedRangeScan * MapperSensorManager::GetScan(const Name & rSensorName, kt_int32s scanIndex)
{
  ScanManager * pScanManager = GetScanManager(rSensorName);
  if (pScanManager != NULL) {
    LocalizedRangeScanMap::iterator it = pScanManager->GetScans().find(scanIndex);
    if (it != pScanManager->GetScans().end()) {
      return it->second;
    } else {
      return nullptr;
    }
  }

  assert(false);
  return NULL;
}

/**
 * Gets last scan of given device
 * @param pLaserRangeFinder
 * @return last localized range scan of device
 */
inline LocalizedRangeScan * MapperSensorManager::GetLastScan(const Name & rSensorName)
{
  RegisterSensor(rSensorName);

  return GetScanManager(rSensorName)->GetLastScan();
}

/**
 * Sets the last scan of device of given scan
 * @param pScan
 */
void MapperSensorManager::SetLastScan(LocalizedRangeScan * pScan)
{
  GetScanManager(pScan)->SetLastScan(pScan);
}

/**
 * Clears the last scan of device of given scan
 * @param pScan
 */
void MapperSensorManager::ClearLastScan(LocalizedRangeScan* pScan)
{
  GetScanManager(pScan)->ClearLastScan();
}

/**
 * Clears the last scan of device name
 * @param pScan
 */
void MapperSensorManager::ClearLastScan(const Name& name)
{
  GetScanManager(name)->ClearLastScan();
}

/**
 * Adds scan to scan vector of device that recorded scan
 * @param pScan
 */
void MapperSensorManager::AddScan(LocalizedRangeScan * pScan)
{
  GetScanManager(pScan)->AddScan(pScan, m_NextScanId);
  m_Scans.insert({m_NextScanId, pScan});
  m_NextScanId++;
}

/**
 * Adds scan to running scans of device that recorded scan
 * @param pScan
 */
inline void MapperSensorManager::AddRunningScan(LocalizedRangeScan * pScan)
{
  GetScanManager(pScan)->AddRunningScan(pScan);
}

/**
 * Finds and replaces a scan from m_Scans with NULL
 * @param pScan
 */
void MapperSensorManager::RemoveScan(LocalizedRangeScan * pScan)
{
  GetScanManager(pScan)->RemoveScan(pScan);

  LocalizedRangeScanMap::iterator it = m_Scans.find(pScan->GetUniqueId());
  if (it != m_Scans.end()) {
    it->second = NULL;
    m_Scans.erase(it);
  } else {
    std::cout << "RemoveScan: Failed to find scan in m_Scans" << std::endl;
  }
}

/**
 * Gets scans of device
 * @param rSensorName
 * @return scans of device
 */
inline LocalizedRangeScanMap & MapperSensorManager::GetScans(const Name & rSensorName)
{
  return GetScanManager(rSensorName)->GetScans();
}

/**
 * Gets running scans of device
 * @param rSensorName
 * @return running scans of device
 */
inline LocalizedRangeScanVector & MapperSensorManager::GetRunningScans(const Name & rSensorName)
{
  return GetScanManager(rSensorName)->GetRunningScans();
}

void MapperSensorManager::ClearRunningScans(const Name & rSensorName)
{
  GetScanManager(rSensorName)->ClearRunningScans();
}

inline kt_int32u MapperSensorManager::GetRunningScanBufferSize(const Name & rSensorName)
{
  return GetScanManager(rSensorName)->GetRunningScanBufferSize();
}

void MapperSensorManager::SetRunningScanBufferSize(kt_int32u rScanBufferSize)
{
  m_RunningBufferMaximumSize = rScanBufferSize;

  std::vector<Name> names = GetSensorNames();
  for (uint i = 0; i != names.size(); i++) {
    GetScanManager(names[i])->SetRunningScanBufferSize(rScanBufferSize);
  }
}

void MapperSensorManager::SetRunningScanBufferMaximumDistance(kt_double rScanBufferMaxDistance)
{
  m_RunningBufferMaximumDistance = rScanBufferMaxDistance;

  std::vector<Name> names = GetSensorNames();
  for (uint i = 0; i != names.size(); i++) {
    GetScanManager(names[i])->SetRunningScanBufferMaximumDistance(rScanBufferMaxDistance);
  }
}

/**
 * Gets all scans of all devices
 * @return all scans of all devices
 */
LocalizedRangeScanVector MapperSensorManager::GetAllScans()
{
  LocalizedRangeScanVector scans;

  forEach(ScanManagerMap, &m_ScanManagers)
  {
    LocalizedRangeScanMap & rScans = iter->second->GetScans();

    LocalizedRangeScanMap::iterator it;
    for (it = rScans.begin(); it != rScans.end(); ++it) {
      scans.push_back(it->second);
    }
  }

  return scans;
}

/**
 * Deletes all scan managers of all devices
 */
void MapperSensorManager::Clear()
{
//    SensorManager::Clear();

  forEach(ScanManagerMap, &m_ScanManagers)
  {
    delete iter->second;
    iter->second = nullptr;
  }

  m_ScanManagers.clear();
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

ScanMatcher::~ScanMatcher()
{
  if (m_pCorrelationGrid) {
    delete m_pCorrelationGrid;
  }
  if (m_pSearchSpaceProbs) {
    delete m_pSearchSpaceProbs;
  }
  if (m_pGridLookup) {
    delete m_pGridLookup;
  }
  if (m_pReflectorGrid) {
    delete m_pReflectorGrid;
  }
}

ScanMatcher * ScanMatcher::Create(
  Mapper * pMapper, kt_double searchSize, kt_double resolution,
  kt_double smearDeviation, kt_double rangeThreshold)
{
  // invalid parameters
  if (resolution <= 0) {
    return NULL;
  }
  if (searchSize <= 0) {
    return NULL;
  }
  if (smearDeviation < 0) {
    return NULL;
  }
  if (rangeThreshold <= 0) {
    return NULL;
  }

  assert(math::DoubleEqual(math::Round(searchSize / resolution), (searchSize / resolution)));

  // calculate search space in grid coordinates
  kt_int32u searchSpaceSideSize = static_cast<kt_int32u>(math::Round(searchSize / resolution) + 1);

  // compute requisite size of correlation grid (pad grid so that scan
  // points can't fall off the grid
  // if a scan is on the border of the search space)
  kt_int32u pointReadingMargin = static_cast<kt_int32u>(ceil(rangeThreshold / resolution));

  kt_int32s gridSize = searchSpaceSideSize + 2 * pointReadingMargin;

  // create correlation grid
  assert(gridSize % 2 == 1);
  CorrelationGrid * pCorrelationGrid = CorrelationGrid::CreateGrid(gridSize, gridSize, resolution,
      smearDeviation);

  // create search space probabilities
  Grid<kt_double> * pSearchSpaceProbs = Grid<kt_double>::CreateGrid(searchSpaceSideSize,
      searchSpaceSideSize, resolution);

  ScanMatcher * pScanMatcher = new ScanMatcher(pMapper);
  pScanMatcher->m_pCorrelationGrid = pCorrelationGrid;
  pScanMatcher->m_pSearchSpaceProbs = pSearchSpaceProbs;
  pScanMatcher->m_pGridLookup = new GridIndexLookup<kt_int8u>(pCorrelationGrid);

  return pScanMatcher;
}

/**
 * Match given scan against set of scans
 * @param pScan scan being scan-matched
 * @param rBaseScans set of scans whose points will mark cells in grid as being occupied
 * @param rMean output parameter of mean (best pose) of match
 * @param rCovariance output parameter of covariance of match
 * @param doPenalize whether to penalize matches further from the search center
 * @param doRefineMatch whether to do finer-grained matching if coarse match is good (default is true)
 * @return strength of response
 */
template<class T>
kt_double ScanMatcher::MatchScan(
  LocalizedRangeScan * pScan, const T & rBaseScans, Pose2 & rMean,
  Matrix3 & rCovariance, kt_bool doPenalize, kt_bool doRefineMatch)
{
  ///////////////////////////////////////
  // set scan pose to be center of grid

  // 1. get scan position
  Pose2 scanPose = pScan->GetSensorPose();

  // scan has no readings; cannot do scan matching
  // best guess of pose is based off of adjusted odometer reading
  if (pScan->GetNumberOfRangeReadings() == 0) {
    rMean = scanPose;

    // maximum covariance
    rCovariance(0, 0) = MAX_VARIANCE;    // XX
    rCovariance(1, 1) = MAX_VARIANCE;    // YY
    rCovariance(2, 2) =
      4 * math::Square(m_pMapper->m_pCoarseAngleResolution->GetValue());    // TH*TH

    return 0.0;
  }

  // 2. get size of grid
  Rectangle2<kt_int32s> roi = m_pCorrelationGrid->GetROI();

  // 3. compute offset (in meters - lower left corner)
  Vector2<kt_double> offset;
  offset.SetX(scanPose.GetX() - (0.5 * (roi.GetWidth() - 1) * m_pCorrelationGrid->GetResolution()));
  offset.SetY(scanPose.GetY() -
    (0.5 * (roi.GetHeight() - 1) * m_pCorrelationGrid->GetResolution()));

  // 4. set offset
  m_pCorrelationGrid->GetCoordinateConverter()->SetOffset(offset);

  ///////////////////////////////////////

  // Reflector-only mode leaves a matcher without reflectors (the loop matcher) nothing to
  // match on: report no match, so loop closure is skipped (REFLECTOR_SLAM.md §5.6).
  m_UseGeometry = m_pMapper->m_pUseGeometry->GetValue() ||
    !m_pMapper->m_pUseReflectors->GetValue();
  PrepareReflectors(pScan);
  if (m_pReflectorGrid) {
    m_pReflectorGrid->GetCoordinateConverter()->SetOffset(offset);
  }
  if (!m_UseGeometry && !ReflectorsAllowed()) {
    rMean = scanPose;
    rCovariance.SetToIdentity();
    rCovariance(0, 0) = MAX_VARIANCE;
    rCovariance(1, 1) = MAX_VARIANCE;
    rCovariance(2, 2) = 4 * math::Square(m_pMapper->m_pCoarseAngleResolution->GetValue());
    return 0.0;
  }

  // set up correlation grid
  AddScans(rBaseScans, scanPose.GetPosition());
  m_ReflectorsActive = m_ReflectorsWanted && m_ReferenceReflectorCount > 0;

  // compute how far to search in each direction
  Vector2<kt_double> searchDimensions(m_pSearchSpaceProbs->GetWidth(),
    m_pSearchSpaceProbs->GetHeight());
  Vector2<kt_double> coarseSearchOffset(0.5 * (searchDimensions.GetX() - 1) *
    m_pCorrelationGrid->GetResolution(),
    0.5 * (searchDimensions.GetY() - 1) * m_pCorrelationGrid->GetResolution());

  // a coarse search only checks half the cells in each dimension
  Vector2<kt_double> coarseSearchResolution(2 * m_pCorrelationGrid->GetResolution(),
    2 * m_pCorrelationGrid->GetResolution());

  // The online sequential match (Process): diagnostics and, with OdometryPriorFusion, the
  // odometry prior (REFLECTOR_SLAM.md 5.10). The prior needs the full 3x3 covariance of the
  // geometry surface, so it always goes through the information path.
  const kt_bool sequentialOnline = doPenalize && this == m_pMapper->m_pSequentialScanMatcher;
  const kt_bool odometryPrior = sequentialOnline &&
    m_pMapper->m_pOdometryPriorFusion->GetValue();
  if (sequentialOnline) {
    m_LastMatch = MatchDiagnostics();
  }
  if (m_UseGeometry && ((m_ReflectorsActive &&
    m_pMapper->m_pReflectorInformationFusion->GetValue()) || odometryPrior))
  {
    const kt_double response = MatchInformation(pScan, scanPose, coarseSearchOffset,
        coarseSearchResolution, doPenalize, doRefineMatch, rMean, rCovariance);
    if (response >= 0.0) {
      if (sequentialOnline) {
        FinishSequentialMatch(pScan, scanPose, rMean, rCovariance);
      }
      return response;
    }
  }

  // L1 (REFLECTOR_SLAM.md §5.9): the loop matcher's coarse covariance from the relative-window
  // 3x3 moments, so a ridge along a featureless corridor reports its full length and fails
  // loop_match_maximum_variance_coarse.
  const MapperGraph * pGraph = m_pMapper->GetGraph();
  const kt_bool relativeCovariance = !doRefineMatch && pGraph != nullptr &&
    this == pGraph->GetLoopScanMatcher() &&
    m_pMapper->m_pLoopMatchRelativeCovariance->GetValue();
  m_ComputeFullCovariance = relativeCovariance;

  // actual scan-matching
  kt_double bestResponse = CorrelateScan(pScan, scanPose, coarseSearchOffset,
      coarseSearchResolution,
      m_pMapper->m_pCoarseSearchAngleOffset->GetValue(),
      m_pMapper->m_pCoarseAngleResolution->GetValue(),
      doPenalize, rMean, rCovariance, false);

  // Without geometry, response 0 just means no reflector in view: widening cannot help.
  if (m_pMapper->m_pUseResponseExpansion->GetValue() == true && m_UseGeometry) {
    if (math::DoubleEqual(bestResponse, 0.0)) {
#ifdef KARTO_DEBUG
      std::cout << "Mapper Info: Expanding response search space!" << std::endl;
#endif
      // try and increase search angle offset with 20 degrees and do another match
      kt_double newSearchAngleOffset = m_pMapper->m_pCoarseSearchAngleOffset->GetValue();
      for (kt_int32u i = 0; i < 3; i++) {
        newSearchAngleOffset += math::DegreesToRadians(20);

        bestResponse = CorrelateScan(pScan, scanPose, coarseSearchOffset, coarseSearchResolution,
            newSearchAngleOffset, m_pMapper->m_pCoarseAngleResolution->GetValue(),
            doPenalize, rMean, rCovariance, false);

        if (math::DoubleEqual(bestResponse, 0.0) == false) {
          break;
        }
      }

#ifdef KARTO_DEBUG
      if (math::DoubleEqual(bestResponse, 0.0)) {
        std::cout << "Mapper Warning: Unable to calculate response!" << std::endl;
      }
#endif
    }
  }

  if (relativeCovariance) {
    m_ComputeFullCovariance = false;
    if (bestResponse > KT_TOLERANCE) {
      rCovariance = m_FullCovariance;
    }
  }

  if (doRefineMatch) {
    Vector2<kt_double> fineSearchOffset(coarseSearchResolution * 0.5);
    Vector2<kt_double> fineSearchResolution(m_pCorrelationGrid->GetResolution(),
      m_pCorrelationGrid->GetResolution());
    bestResponse = CorrelateScan(pScan, rMean, fineSearchOffset, fineSearchResolution,
        0.5 * m_pMapper->m_pCoarseAngleResolution->GetValue(),
        m_pMapper->m_pFineSearchAngleOffset->GetValue(),
        doPenalize, rMean, rCovariance, true);
  }

  // One reflector fixes position but not heading: the score is a ridge of poses rotating about
  // it. The odometry penalty picks the right one, but the angular variance, taken at a fixed
  // cell, comes out small. Without geometry and with fewer than two matched reflectors, fall
  // back to the variance used for response 0 (REFLECTOR_SLAM.md §5.6).
  const kt_int32u matchedReflectors = CountMatchedReflectors(pScan, rMean);
  if (!m_UseGeometry && matchedReflectors < 2) {
    rCovariance(2, 2) = math::Maximum(rCovariance(2, 2),
        4 * math::Square(m_pMapper->m_pCoarseAngleResolution->GetValue()));
  }
  if (doPenalize) {
    m_LastReflectorStats.fused = false;
    m_LastReflectorStats.gated = false;
    m_LastReflectorStats.gateD2 = 0.0;
    m_LastReflectorStats.active = m_ReflectorsActive;
    m_LastReflectorStats.scanPoints = static_cast<kt_int32u>(pScan->GetReflectorPoints().size());
    m_LastReflectorStats.referencePoints = m_ReferenceReflectorCount;
    m_LastReflectorStats.matched = matchedReflectors;
    m_LastReflectorStats.response = bestResponse;
  }

#ifdef KARTO_DEBUG
  std::cout << "  BEST POSE = " << rMean << " BEST RESPONSE = " << bestResponse <<
    ",  VARIANCE = " <<
    rCovariance(0, 0) << ", " << rCovariance(1, 1) << std::endl;
#endif
  assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

  if (sequentialOnline) {
    FinishSequentialMatch(pScan, scanPose, rMean, rCovariance);
  }
  return bestResponse;
}

kt_double ScanMatcher::Penalty(kt_double squaredDistance, kt_double angleOffset) const
{
  kt_double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN *
    squaredDistance / m_pMapper->m_pDistanceVariancePenalty->GetValue());
  distancePenalty = math::Maximum(distancePenalty,
      m_pMapper->m_pMinimumDistancePenalty->GetValue());

  kt_double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN *
    math::Square(angleOffset) / m_pMapper->m_pAngleVariancePenalty->GetValue());
  anglePenalty = math::Maximum(anglePenalty, m_pMapper->m_pMinimumAnglePenalty->GetValue());
  return distancePenalty * anglePenalty;
}

void ScanMatcher::operator()(const kt_double & y) const
{
  kt_int32u poseResponseCounter;
  kt_int32u x_pose;
  kt_int32u y_pose = std::find(m_yPoses.begin(), m_yPoses.end(), y) - m_yPoses.begin();

  const kt_int32u size_x = m_xPoses.size();

  kt_double newPositionY = m_rSearchCenter.GetY() + y;
  kt_double squareY = math::Square(y);

  for (std::vector<kt_double>::const_iterator xIter = m_xPoses.begin(); xIter != m_xPoses.end();
    ++xIter)
  {
    x_pose = std::distance(m_xPoses.begin(), xIter);
    kt_double x = *xIter;
    kt_double newPositionX = m_rSearchCenter.GetX() + x;
    kt_double squareX = math::Square(x);

    Vector2<kt_int32s> gridPoint =
      m_pCorrelationGrid->WorldToGrid(Vector2<kt_double>(newPositionX, newPositionY));
    kt_int32s gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);
    assert(gridIndex >= 0);
    const kt_int32s reflectorGridIndex =
      ReflectorGridIndex(Vector2<kt_double>(newPositionX, newPositionY));

    kt_double angle = 0.0;
    kt_double startAngle = m_rSearchCenter.GetHeading() - m_searchAngleOffset;
    for (kt_int32u angleIndex = 0; angleIndex < m_nAngles; angleIndex++) {
      angle = startAngle + angleIndex * m_searchAngleResolution;

      kt_double accept = 0.0;
      kt_double response = GetScore(angleIndex, gridIndex, reflectorGridIndex, accept);
      if (m_doPenalize && (math::DoubleEqual(response, 0.0) == false)) {
        // simple model (approximate Gaussian) to take odometry into account
        const kt_double penalty = Penalty(squareX + squareY, angle - m_rSearchCenter.GetHeading());
        response *= penalty;
        accept *= penalty;
      }

      // store response and pose
      poseResponseCounter = (y_pose * size_x + x_pose) * (m_nAngles) + angleIndex;
      m_pAcceptResponse[poseResponseCounter] = accept;
      m_pPoseResponse[poseResponseCounter] =
        std::pair<kt_double, Pose2>(response, Pose2(newPositionX, newPositionY,
          math::NormalizeAngle(angle)));
    }
  }
}

/**
 * Finds the best pose for the scan centering the search in the correlation grid
 * at the given pose and search in the space by the vector and angular offsets
 * in increments of the given resolutions
 * @param rScan scan to match against correlation grid
 * @param rSearchCenter the center of the search space
 * @param rSearchSpaceOffset searches poses in the area offset by this vector around search center
 * @param rSearchSpaceResolution how fine a granularity to search in the search space
 * @param searchAngleOffset searches poses in the angles offset by this angle around search center
 * @param searchAngleResolution how fine a granularity to search in the angular search space
 * @param doPenalize whether to penalize matches further from the search center
 * @param rMean output parameter of mean (best pose) of match
 * @param rCovariance output parameter of covariance of match
 * @param doingFineMatch whether to do a finer search after coarse search
 * @return strength of response
 */
kt_double ScanMatcher::CorrelateScan(
  LocalizedRangeScan * pScan, const Pose2 & rSearchCenter,
  const Vector2<kt_double> & rSearchSpaceOffset,
  const Vector2<kt_double> & rSearchSpaceResolution,
  kt_double searchAngleOffset, kt_double searchAngleResolution,
  kt_bool doPenalize, Pose2 & rMean, Matrix3 & rCovariance, kt_bool doingFineMatch)
{
  assert(searchAngleResolution != 0.0);

  // setup lookup arrays
  m_pGridLookup->ComputeOffsets(pScan,
    rSearchCenter.GetHeading(), searchAngleOffset, searchAngleResolution);
  if (m_ReflectorsActive) {
    ComputeReflectorOffsets(pScan,
      rSearchCenter.GetHeading(), searchAngleOffset, searchAngleResolution);
  }

  // only initialize probability grid if computing positional covariance (during coarse match)
  if (!doingFineMatch) {
    m_pSearchSpaceProbs->Clear();

    // position search grid - finds lower left corner of search grid
    Vector2<kt_double> offset(rSearchCenter.GetPosition() - rSearchSpaceOffset);
    m_pSearchSpaceProbs->GetCoordinateConverter()->SetOffset(offset);
  }

  // calculate position arrays

  m_xPoses.clear();
  kt_int32u nX = static_cast<kt_int32u>(math::Round(rSearchSpaceOffset.GetX() *
    2.0 / rSearchSpaceResolution.GetX()) + 1);
  kt_double startX = -rSearchSpaceOffset.GetX();
  for (kt_int32u xIndex = 0; xIndex < nX; xIndex++) {
    m_xPoses.push_back(startX + xIndex * rSearchSpaceResolution.GetX());
  }
  assert(math::DoubleEqual(m_xPoses.back(), -startX));

  m_yPoses.clear();
  kt_int32u nY = static_cast<kt_int32u>(math::Round(rSearchSpaceOffset.GetY() *
    2.0 / rSearchSpaceResolution.GetY()) + 1);
  kt_double startY = -rSearchSpaceOffset.GetY();
  for (kt_int32u yIndex = 0; yIndex < nY; yIndex++) {
    m_yPoses.push_back(startY + yIndex * rSearchSpaceResolution.GetY());
  }
  assert(math::DoubleEqual(m_yPoses.back(), -startY));

  // calculate pose response array size
  kt_int32u nAngles =
    static_cast<kt_int32u>(math::Round(searchAngleOffset * 2.0 / searchAngleResolution) + 1);

  kt_int32u poseResponseSize = static_cast<kt_int32u>(m_xPoses.size() * m_yPoses.size() * nAngles);

  // allocate array
  m_pPoseResponse = new std::pair<kt_double, Pose2>[poseResponseSize];
  m_pAcceptResponse = new kt_double[poseResponseSize];

  Vector2<kt_int32s> startGridPoint =
    m_pCorrelationGrid->WorldToGrid(Vector2<kt_double>(rSearchCenter.GetX() +
      startX, rSearchCenter.GetY() + startY));

  // this isn't good but its the fastest way to iterate. Should clean up later.
  m_rSearchCenter = rSearchCenter;
  m_searchAngleOffset = searchAngleOffset;
  m_nAngles = nAngles;
  m_searchAngleResolution = searchAngleResolution;
  m_doPenalize = doPenalize;
  tbb::parallel_for_each(m_yPoses, (*this));

  // find value of best response (in [0; 1])
  kt_double bestResponse = -1;
  for (kt_int32u i = 0; i < poseResponseSize; i++) {
    bestResponse = math::Maximum(bestResponse, m_pPoseResponse[i].first);

    // will compute positional covariance, save best relative probability for each cell
    if (!doingFineMatch) {
      const Pose2 & rPose = m_pPoseResponse[i].second;
      Vector2<kt_int32s> grid = m_pSearchSpaceProbs->WorldToGrid(rPose.GetPosition());
      kt_double * ptr;

      try {
        ptr = (kt_double *)(m_pSearchSpaceProbs->GetDataPointer(grid));  // NOLINT
      } catch (...) {
        throw std::runtime_error("Mapper FATAL ERROR - "
                "unable to get pointer in probability search!");
      }

      if (ptr == NULL) {
        throw std::runtime_error("Mapper FATAL ERROR - "
                "Index out of range in probability search!");
      }

      *ptr = math::Maximum(m_pPoseResponse[i].first, *ptr);
    }
  }

  // Diagnostics (REFLECTOR_SLAM.md 5.10, S0): every candidate with its penalized and raw response.
  if (m_pSurfaceDump != nullptr) {
    std::ostream & out = *m_pSurfaceDump;
    for (kt_int32u i = 0; i < poseResponseSize; i++) {
      const Pose2 & p = m_pPoseResponse[i].second;
      const kt_double response = m_pPoseResponse[i].first;
      const kt_double penalty = m_doPenalize ? Penalty(
        math::Square(p.GetX() - rSearchCenter.GetX()) + math::Square(p.GetY() - rSearchCenter.GetY()),
        math::NormalizeAngle(p.GetHeading() - rSearchCenter.GetHeading())) : 1.0;
      out << pScan->GetTime() << ' ' << (doingFineMatch ? 1 : 0) << ' ' <<
        static_cast<int>(m_ScoreSource) << ' ' << rSearchCenter.GetX() << ' ' <<
        rSearchCenter.GetY() << ' ' << rSearchCenter.GetHeading() << ' ' << p.GetX() << ' ' <<
        p.GetY() << ' ' << p.GetHeading() << ' ' << response << ' ' <<
        (response != 0.0 ? response / penalty : 0.0) << '\n';
    }
  }

  // average all poses with same highest response
  Vector2<kt_double> averagePosition;
  kt_double thetaX = 0.0;
  kt_double thetaY = 0.0;
  kt_int32s averagePoseCount = 0;
  // Acceptance response of the best poses; equal to bestResponse while reflectors are inactive.
  kt_double bestAccept = 0.0;
  for (kt_int32u i = 0; i < poseResponseSize; i++) {
    if (math::DoubleEqual(m_pPoseResponse[i].first, bestResponse)) {
      bestAccept = math::Maximum(bestAccept, m_pAcceptResponse[i]);
      averagePosition += m_pPoseResponse[i].second.GetPosition();

      kt_double heading = m_pPoseResponse[i].second.GetHeading();
      thetaX += cos(heading);
      thetaY += sin(heading);

      averagePoseCount++;
    }
  }

  Pose2 averagePose;
  if (averagePoseCount > 0) {
    averagePosition /= averagePoseCount;

    thetaX /= averagePoseCount;
    thetaY /= averagePoseCount;

    averagePose = Pose2(averagePosition, atan2(thetaY, thetaX));
  } else {
    throw std::runtime_error("Mapper FATAL ERROR - Unable to find best position");
  }

  // Full 3x3 covariance over the whole coarse search (REFLECTOR_SLAM.md §5.5): weighted second
  // moments of the poses scoring at least rho times the best, so it no longer depends on how the
  // scores are scaled. Floors keep it invertible: half a search step in position and angle.
  if (m_ComputeFullCovariance && !doingFineMatch) {
    m_FullCovariance.SetToIdentity();
    const kt_double rho = m_pMapper->m_pReflectorCovarianceWindow->GetValue();
    kt_double sum = 0.0;
    kt_double moments[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    if (bestResponse > KT_TOLERANCE) {
      for (kt_int32u i = 0; i < poseResponseSize; i++) {
        const kt_double response = m_pPoseResponse[i].first;
        if (response < rho * bestResponse) {
          continue;
        }
        const Pose2 & rPose = m_pPoseResponse[i].second;
        const kt_double d[3] = {rPose.GetX() - averagePose.GetX(),
          rPose.GetY() - averagePose.GetY(),
          math::NormalizeAngle(rPose.GetHeading() - averagePose.GetHeading())};
        for (int r = 0; r < 3; r++) {
          for (int c = 0; c < 3; c++) {
            moments[r][c] += response * d[r] * d[c];
          }
        }
        sum += response;
      }
    }
    if (sum > KT_TOLERANCE) {
      for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
          m_FullCovariance(r, c) = moments[r][c] / sum;
        }
      }
    } else {
      m_FullCovariance(0, 0) = MAX_VARIANCE;
      m_FullCovariance(1, 1) = MAX_VARIANCE;
      m_FullCovariance(2, 2) = math::Square(searchAngleOffset);
    }
    m_FullCovariance(0, 0) = math::Maximum(m_FullCovariance(0, 0),
        0.25 * math::Square(rSearchSpaceResolution.GetX()));
    m_FullCovariance(1, 1) = math::Maximum(m_FullCovariance(1, 1),
        0.25 * math::Square(rSearchSpaceResolution.GetY()));
    m_FullCovariance(2, 2) = math::Maximum(m_FullCovariance(2, 2),
        0.25 * math::Square(searchAngleResolution));
  }

  // delete pose response array
  delete[] m_pPoseResponse;
  m_pPoseResponse = nullptr;
  delete[] m_pAcceptResponse;
  m_pAcceptResponse = nullptr;

#ifdef KARTO_DEBUG
  std::cout << "bestPose: " << averagePose << std::endl;
  std::cout << "bestResponse: " << bestResponse << std::endl;
#endif

  if (!doingFineMatch) {
    ComputePositionalCovariance(averagePose, bestResponse, rSearchCenter, rSearchSpaceOffset,
      rSearchSpaceResolution, searchAngleResolution, rCovariance);
  } else {
    ComputeAngularCovariance(averagePose, bestResponse, rSearchCenter,
      searchAngleOffset, searchAngleResolution, rCovariance);
  }

  rMean = averagePose;

#ifdef KARTO_DEBUG
  std::cout << "bestPose: " << averagePose << std::endl;
#endif

  if (bestResponse > 1.0) {
    bestResponse = 1.0;
  }

  assert(math::InRange(bestResponse, 0.0, 1.0));
  assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

  // Accept on R_geom (R_refl without geometry), so reflectors cannot rescue a poor geometric
  // match; pose and covariance come from the combined score above.
  if (!m_ReflectorsActive) {
    return bestResponse;
  }
  return math::Minimum(bestAccept, 1.0);
}

/**
 * Computes the positional covariance of the best pose
 * @param rBestPose
 * @param bestResponse
 * @param rSearchCenter
 * @param rSearchSpaceOffset
 * @param rSearchSpaceResolution
 * @param searchAngleResolution
 * @param rCovariance
 */
void ScanMatcher::ComputePositionalCovariance(
  const Pose2 & rBestPose, kt_double bestResponse,
  const Pose2 & rSearchCenter,
  const Vector2<kt_double> & rSearchSpaceOffset,
  const Vector2<kt_double> & rSearchSpaceResolution,
  kt_double searchAngleResolution, Matrix3 & rCovariance)
{
  // reset covariance to identity matrix
  rCovariance.SetToIdentity();

  // if best response is vary small return max variance
  if (bestResponse < KT_TOLERANCE) {
    rCovariance(0, 0) = MAX_VARIANCE;    // XX
    rCovariance(1, 1) = MAX_VARIANCE;    // YY
    rCovariance(2, 2) = 4 * math::Square(searchAngleResolution);    // TH*TH

    return;
  }

  kt_double accumulatedVarianceXX = 0;
  kt_double accumulatedVarianceXY = 0;
  kt_double accumulatedVarianceYY = 0;
  kt_double norm = 0;

  kt_double dx = rBestPose.GetX() - rSearchCenter.GetX();
  kt_double dy = rBestPose.GetY() - rSearchCenter.GetY();

  kt_double offsetX = rSearchSpaceOffset.GetX();
  kt_double offsetY = rSearchSpaceOffset.GetY();

  kt_int32u nX =
    static_cast<kt_int32u>(math::Round(offsetX * 2.0 / rSearchSpaceResolution.GetX()) + 1);
  kt_double startX = -offsetX;
  assert(math::DoubleEqual(startX + (nX - 1) * rSearchSpaceResolution.GetX(), -startX));

  kt_int32u nY =
    static_cast<kt_int32u>(math::Round(offsetY * 2.0 / rSearchSpaceResolution.GetY()) + 1);
  kt_double startY = -offsetY;
  assert(math::DoubleEqual(startY + (nY - 1) * rSearchSpaceResolution.GetY(), -startY));

  for (kt_int32u yIndex = 0; yIndex < nY; yIndex++) {
    kt_double y = startY + yIndex * rSearchSpaceResolution.GetY();

    for (kt_int32u xIndex = 0; xIndex < nX; xIndex++) {
      kt_double x = startX + xIndex * rSearchSpaceResolution.GetX();

      Vector2<kt_int32s> gridPoint =
        m_pSearchSpaceProbs->WorldToGrid(Vector2<kt_double>(rSearchCenter.GetX() + x,
          rSearchCenter.GetY() + y));
      kt_double response = *(m_pSearchSpaceProbs->GetDataPointer(gridPoint));

      // response is not a low response
      if (response >= (bestResponse - 0.1)) {
        norm += response;
        accumulatedVarianceXX += (math::Square(x - dx) * response);
        accumulatedVarianceXY += ((x - dx) * (y - dy) * response);
        accumulatedVarianceYY += (math::Square(y - dy) * response);
      }
    }
  }

  if (norm > KT_TOLERANCE) {
    kt_double varianceXX = accumulatedVarianceXX / norm;
    kt_double varianceXY = accumulatedVarianceXY / norm;
    kt_double varianceYY = accumulatedVarianceYY / norm;
    kt_double varianceTHTH = 4 * math::Square(searchAngleResolution);

    // lower-bound variances so that they are not too small;
    // ensures that links are not too tight
    kt_double minVarianceXX = 0.1 * math::Square(rSearchSpaceResolution.GetX());
    kt_double minVarianceYY = 0.1 * math::Square(rSearchSpaceResolution.GetY());
    varianceXX = math::Maximum(varianceXX, minVarianceXX);
    varianceYY = math::Maximum(varianceYY, minVarianceYY);

    // increase variance for poorer responses
    kt_double multiplier = 1.0 / bestResponse;
    rCovariance(0, 0) = varianceXX * multiplier;
    rCovariance(0, 1) = varianceXY * multiplier;
    rCovariance(1, 0) = varianceXY * multiplier;
    rCovariance(1, 1) = varianceYY * multiplier;
    rCovariance(2, 2) = varianceTHTH;    // this value will be set in ComputeAngularCovariance
  }

  // if values are 0, set to MAX_VARIANCE
  // values might be 0 if points are too sparse and thus don't hit other points
  if (math::DoubleEqual(rCovariance(0, 0), 0.0)) {
    rCovariance(0, 0) = MAX_VARIANCE;
  }

  if (math::DoubleEqual(rCovariance(1, 1), 0.0)) {
    rCovariance(1, 1) = MAX_VARIANCE;
  }
}

/**
 * Computes the angular covariance of the best pose
 * @param rBestPose
 * @param bestResponse
 * @param rSearchCenter
 * @param rSearchAngleOffset
 * @param searchAngleResolution
 * @param rCovariance
 */
void ScanMatcher::ComputeAngularCovariance(
  const Pose2 & rBestPose,
  kt_double bestResponse,
  const Pose2 & rSearchCenter,
  kt_double searchAngleOffset,
  kt_double searchAngleResolution,
  Matrix3 & rCovariance)
{
  // NOTE: do not reset covariance matrix

  // normalize angle difference
  kt_double bestAngle = math::NormalizeAngleDifference(
    rBestPose.GetHeading(), rSearchCenter.GetHeading());

  Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(rBestPose.GetPosition());
  kt_int32s gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);
  const kt_int32s reflectorGridIndex = ReflectorGridIndex(rBestPose.GetPosition());

  kt_int32u nAngles =
    static_cast<kt_int32u>(math::Round(searchAngleOffset * 2 / searchAngleResolution) + 1);

  kt_double angle = 0.0;
  kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;

  kt_double norm = 0.0;
  kt_double accumulatedVarianceThTh = 0.0;
  for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++) {
    angle = startAngle + angleIndex * searchAngleResolution;
    kt_double accept = 0.0;
    kt_double response = GetScore(angleIndex, gridIndex, reflectorGridIndex, accept);

    // response is not a low response
    if (response >= (bestResponse - 0.1)) {
      norm += response;
      accumulatedVarianceThTh += (math::Square(angle - bestAngle) * response);
    }
  }
  assert(math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));

  if (norm > KT_TOLERANCE) {
    if (accumulatedVarianceThTh < KT_TOLERANCE) {
      accumulatedVarianceThTh = math::Square(searchAngleResolution);
    }

    accumulatedVarianceThTh /= norm;
  } else {
    accumulatedVarianceThTh = 1000 * math::Square(searchAngleResolution);
  }

  rCovariance(2, 2) = accumulatedVarianceThTh;
}

/**
 * Marks cells where scans' points hit as being occupied
 * @param rScans scans whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 */
void ScanMatcher::AddScans(const LocalizedRangeScanVector & rScans, Vector2<kt_double> viewPoint)
{
  m_pCorrelationGrid->Clear();
  m_ReferenceReflectorCount = 0;

  // add all scans to grid
  const_forEach(LocalizedRangeScanVector, &rScans)
  {
    if (*iter == NULL) {
      continue;
    }

    AddScan(*iter, viewPoint);
  }
}

/**
 * Marks cells where scans' points hit as being occupied
 * @param rScans scans whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 */
void ScanMatcher::AddScans(const LocalizedRangeScanMap & rScans, Vector2<kt_double> viewPoint)
{
  m_pCorrelationGrid->Clear();
  m_ReferenceReflectorCount = 0;

  // add all scans to grid
  const_forEach(LocalizedRangeScanMap, &rScans)
  {
    if (iter->second == NULL) {
      continue;
    }

    AddScan(iter->second, viewPoint);
  }
}

/**
 * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
 * @param pScan scan whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 * @param doSmear whether the points will be smeared
 */
void ScanMatcher::AddScan(
  LocalizedRangeScan * pScan, const Vector2<kt_double> & rViewPoint,
  kt_bool doSmear)
{
  PointVectorDouble validPoints = FindValidPoints(pScan, rViewPoint);

  // put in all valid points
  const_forEach(PointVectorDouble, &validPoints)
  {
    Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(*iter);
    if (!math::IsUpTo(gridPoint.GetX(), m_pCorrelationGrid->GetROI().GetWidth()) ||
      !math::IsUpTo(gridPoint.GetY(), m_pCorrelationGrid->GetROI().GetHeight()))
    {
      // point not in grid
      continue;
    }

    int gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

    // set grid cell as occupied
    if (m_pCorrelationGrid->GetDataPointer()[gridIndex] == GridStates_Occupied) {
      // value already set
      continue;
    }

    m_pCorrelationGrid->GetDataPointer()[gridIndex] = GridStates_Occupied;

    // smear grid
    if (doSmear == true) {
      m_pCorrelationGrid->SmearPoint(gridPoint);
    }
  }

  if (!m_ReflectorsWanted) {
    return;
  }
  // Reference reflector points. No view-point filter: markers are few and on walls.
  const PointVectorDouble & rReflectorPoints = pScan->GetReflectorPoints();
  const Rectangle2<kt_int32s> & roi = m_pReflectorGrid->GetROI();
  for (const Vector2<kt_double> & rPoint : rReflectorPoints) {
    Vector2<kt_int32s> gridPoint = m_pReflectorGrid->WorldToGrid(rPoint);
    if (!math::IsUpTo(gridPoint.GetX(), roi.GetWidth()) ||
      !math::IsUpTo(gridPoint.GetY(), roi.GetHeight()))
    {
      continue;
    }
    const int gridIndex = m_pReflectorGrid->GridIndex(gridPoint);
    if (m_pReflectorGrid->GetDataPointer()[gridIndex] != GridStates_Occupied) {
      m_pReflectorGrid->GetDataPointer()[gridIndex] = GridStates_Occupied;
      m_pReflectorGrid->SmearPoint(gridPoint);
    }
    ++m_ReferenceReflectorCount;
  }
}

/**
 * Compute which points in a scan are on the same side as the given viewpoint
 * @param pScan
 * @param rViewPoint
 * @return points on the same side
 */
PointVectorDouble ScanMatcher::FindValidPoints(
  LocalizedRangeScan * pScan,
  const Vector2<kt_double> & rViewPoint) const
{
  const PointVectorDouble & rPointReadings = pScan->GetPointReadings();

  // points must be at least 10 cm away when making comparisons of inside/outside of viewpoint
  const kt_double minSquareDistance = math::Square(0.1);    // in m^2

  // this iterator lags from the main iterator adding points only when the points are on
  // the same side as the viewpoint
  PointVectorDouble::const_iterator trailingPointIter = rPointReadings.begin();
  PointVectorDouble validPoints;

  Vector2<kt_double> firstPoint;
  kt_bool firstTime = true;
  const_forEach(PointVectorDouble, &rPointReadings)
  {
    Vector2<kt_double> currentPoint = *iter;

    if (firstTime && !std::isnan(currentPoint.GetX()) && !std::isnan(currentPoint.GetY())) {
      firstPoint = currentPoint;
      firstTime = false;
    }

    Vector2<kt_double> delta = firstPoint - currentPoint;
    if (delta.SquaredLength() > minSquareDistance) {
      // This compute the Determinant (viewPoint FirstPoint, viewPoint currentPoint)
      // Which computes the direction of rotation, if the rotation is counterclock
      // wise then we are looking at data we should keep. If it's negative rotation
      // we should not included in in the matching
      // have enough distance, check viewpoint
      double a = rViewPoint.GetY() - firstPoint.GetY();
      double b = firstPoint.GetX() - rViewPoint.GetX();
      double c = firstPoint.GetY() * rViewPoint.GetX() - firstPoint.GetX() * rViewPoint.GetY();
      double ss = currentPoint.GetX() * a + currentPoint.GetY() * b + c;

      // reset beginning point
      firstPoint = currentPoint;

      if (ss < 0.0) {  // wrong side, skip and keep going
        trailingPointIter = iter;
      } else {
        for (; trailingPointIter != iter; ++trailingPointIter) {
          validPoints.push_back(*trailingPointIter);
        }
      }
    }
  }

  return validPoints;
}

/**
 * Get response at given position for given rotation (only look up valid points)
 * @param angleIndex
 * @param gridPositionIndex
 * @return response
 */
kt_double ScanMatcher::GetResponse(kt_int32u angleIndex, kt_int32s gridPositionIndex) const
{
  kt_double response = 0.0;

  // add up value for each point
  kt_int8u * pByte = m_pCorrelationGrid->GetDataPointer() + gridPositionIndex;

  const LookupArray * pOffsets = m_pGridLookup->GetLookupArray(angleIndex);
  assert(pOffsets != NULL);

  // get number of points in offset list
  kt_int32u nPoints = pOffsets->GetSize();
  if (nPoints == 0) {
    return response;
  }

  // calculate response
  kt_int32s * pAngleIndexPointer = pOffsets->GetArrayPointer();
  for (kt_int32u i = 0; i < nPoints; i++) {
    // ignore points that fall off the grid
    kt_int32s pointGridIndex = gridPositionIndex + pAngleIndexPointer[i];
    if (!math::IsUpTo(pointGridIndex,
      m_pCorrelationGrid->GetDataSize()) || pAngleIndexPointer[i] == INVALID_SCAN)
    {
      continue;
    }

    // uses index offsets to efficiently find location of point in the grid
    response += pByte[pAngleIndexPointer[i]];
  }

  // normalize response
  response /= (nPoints * GridStates_Occupied);
  assert(fabs(response) <= 1.0);

  return response;
}

kt_double ScanMatcher::GetScore(
  kt_int32u angleIndex, kt_int32s gridPositionIndex, kt_int32s reflectorGridIndex,
  kt_double & rAccept) const
{
  if (m_ScoreSource == ScoreSource::Geometry) {
    rAccept = GetResponse(angleIndex, gridPositionIndex);
    return rAccept;
  }
  if (m_ScoreSource == ScoreSource::Reflector) {
    rAccept = GetReflectorResponse(angleIndex, reflectorGridIndex);
    return rAccept;
  }
  const kt_double geometry = m_UseGeometry ? GetResponse(angleIndex, gridPositionIndex) : 0.0;
  if (!m_ReflectorsActive) {
    rAccept = geometry;
    return geometry;
  }
  const kt_double reflector = GetReflectorResponse(angleIndex, reflectorGridIndex);
  const kt_double g = m_UseGeometry ? 1.0 : 0.0;
  rAccept = m_UseGeometry ? geometry : reflector;
  return (g * geometry + m_ReflectorWeight * reflector) / (g + m_ReflectorWeight);
}

kt_double ScanMatcher::GetReflectorResponse(
  kt_int32u angleIndex, kt_int32s reflectorGridIndex) const
{
  const std::vector<kt_int32s> & rOffsets = m_ReflectorOffsets[angleIndex];
  const kt_int8u * pByte = m_pReflectorGrid->GetDataPointer() + reflectorGridIndex;
  kt_double sum = 0.0;
  for (size_t i = 0; i < rOffsets.size(); i++) {
    const kt_int32s pointGridIndex = reflectorGridIndex + rOffsets[i];
    if (rOffsets[i] == INVALID_SCAN ||
      !math::IsUpTo(pointGridIndex, m_pReflectorGrid->GetDataSize()))
    {
      continue;
    }
    sum += m_ReflectorWeights[i] * pByte[rOffsets[i]];
  }
  sum /= GridStates_Occupied;
  return math::Minimum(sum, m_ReflectorCap) / m_ReflectorCap;
}

kt_int32s ScanMatcher::ReflectorGridIndex(const Vector2<kt_double> & rPosition) const
{
  if (!m_ReflectorsActive) {
    return 0;
  }
  return m_pReflectorGrid->GridIndex(m_pReflectorGrid->WorldToGrid(rPosition));
}

kt_bool ScanMatcher::ReflectorsAllowed() const
{
  if (this == m_pMapper->m_pSequentialScanMatcher) {
    return true;
  }
  const MapperGraph * pGraph = m_pMapper->GetGraph();
  return pGraph != nullptr && this == pGraph->GetLoopScanMatcher() &&
         m_pMapper->m_pReflectorsInLoopClosure->GetValue();
}

void ScanMatcher::PrepareReflectors(LocalizedRangeScan * pScan)
{
  m_ReflectorsActive = false;
  m_ReferenceReflectorCount = 0;
  m_ReflectorWeight = m_pMapper->m_pReflectorWeight->GetValue();
  m_ReflectorCap = math::Maximum(m_pMapper->m_pReflectorResponseCap->GetValue(), KT_TOLERANCE);
  m_ReflectorsWanted = m_pMapper->m_pUseReflectors->GetValue() && ReflectorsAllowed() &&
    !pScan->GetReflectorPoints().empty() &&
    (m_ReflectorWeight > 0.0 || !m_UseGeometry);
  if (!m_ReflectorsWanted) {
    return;
  }
  if (!m_UseGeometry) {
    // Only the reflector term counts; its weight just has to be positive.
    m_ReflectorWeight = 1.0;
  }

  // Same extent and resolution as the geometry grid; the smear must stay within the kernel
  // limits of CorrelationGrid (0.5 to 10 cells).
  const kt_double resolution = m_pCorrelationGrid->GetResolution();
  const kt_double smear = math::Clip(m_pMapper->m_pReflectorSmearDeviation->GetValue(),
      0.5 * resolution, 10.0 * resolution);
  if (!m_pReflectorGrid || !math::DoubleEqual(smear, m_ReflectorGridSmear)) {
    delete m_pReflectorGrid;
    const Rectangle2<kt_int32s> & roi = m_pCorrelationGrid->GetROI();
    m_pReflectorGrid = CorrelationGrid::CreateGrid(roi.GetWidth(), roi.GetHeight(), resolution,
        smear);
    m_ReflectorGridSmear = smear;
  }
  m_pReflectorGrid->Clear();
  m_ReflectorWeights = pScan->GetReflectorPointWeights();
}

void ScanMatcher::ComputeReflectorOffsets(
  LocalizedRangeScan * pScan, kt_double angleCenter, kt_double angleOffset,
  kt_double angleResolution)
{
  const kt_int32u nAngles =
    static_cast<kt_int32u>(math::Round(angleOffset * 2.0 / angleResolution) + 1);
  m_ReflectorOffsets.assign(nAngles, std::vector<kt_int32s>());

  // reflector points in the sensor frame
  Transform transform(pScan->GetSensorPose());
  std::vector<Vector2<kt_double>> localPoints;
  for (const Vector2<kt_double> & rPoint : pScan->GetReflectorPoints()) {
    localPoints.push_back(transform.InverseTransformPose(Pose2(rPoint, 0.0)).GetPosition());
  }

  const Vector2<kt_double> & rGridOffset =
    m_pReflectorGrid->GetCoordinateConverter()->GetOffset();
  const kt_double startAngle = angleCenter - angleOffset;
  for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++) {
    const kt_double angle = startAngle + angleIndex * angleResolution;
    const kt_double cosine = cos(angle);
    const kt_double sine = sin(angle);
    std::vector<kt_int32s> & rOffsets = m_ReflectorOffsets[angleIndex];
    for (const Vector2<kt_double> & rPosition : localPoints) {
      Vector2<kt_double> offset;
      offset.SetX(cosine * rPosition.GetX() - sine * rPosition.GetY());
      offset.SetY(sine * rPosition.GetX() + cosine * rPosition.GetY());
      // as GridIndexLookup: relative to the grid offset, base GridIndex to ignore the ROI
      const Vector2<kt_int32s> gridPoint = m_pReflectorGrid->WorldToGrid(offset + rGridOffset);
      rOffsets.push_back(m_pReflectorGrid->Grid<kt_int8u>::GridIndex(gridPoint, false));
    }
  }
}

kt_int32u ScanMatcher::CountMatchedReflectors(
  LocalizedRangeScan * pScan, const Pose2 & rSensorPose) const
{
  if (!m_ReflectorsActive) {
    return 0;
  }
  Transform from(pScan->GetSensorPose());
  Transform to(rSensorPose);
  const Rectangle2<kt_int32s> & roi = m_pReflectorGrid->GetROI();
  kt_int32u matched = 0;
  for (const Vector2<kt_double> & rPoint : pScan->GetReflectorPoints()) {
    const Pose2 local = from.InverseTransformPose(Pose2(rPoint, 0.0));
    const Vector2<kt_int32s> gridPoint =
      m_pReflectorGrid->WorldToGrid(to.TransformPose(local).GetPosition());
    if (!math::IsUpTo(gridPoint.GetX(), roi.GetWidth()) ||
      !math::IsUpTo(gridPoint.GetY(), roi.GetHeight()))
    {
      continue;
    }
    if (m_pReflectorGrid->GetDataPointer()[m_pReflectorGrid->GridIndex(gridPoint)] >=
      GridStates_Occupied / 2)
    {
      ++matched;
    }
  }
  return matched;
}

namespace
{
// Nearest positive definite matrix: eigenvalues raised to floor. For covariances assembled from
// separate pieces (REFLECTOR_SLAM.md §5.5), which rounding can leave slightly indefinite; an
// indefinite one inverts to negative variances and poisons fusion and the pose graph.
Matrix3 PositiveDefinite(const Matrix3 & rCovariance, kt_double floor = 1e-12)
{
  Eigen::Matrix3d m;
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      m(r, c) = 0.5 * (rCovariance(r, c) + rCovariance(c, r));
    }
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(m);
  m = solver.eigenvectors() * solver.eigenvalues().cwiseMax(floor).asDiagonal() *
    solver.eigenvectors().transpose();
  Matrix3 out;
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      out(r, c) = m(r, c);
    }
  }
  return out;
}
}  // namespace

ScanMatcher::SourceEstimate ScanMatcher::EstimateSource(
  ScoreSource source, LocalizedRangeScan * pScan, const Pose2 & rSearchCenter,
  const Vector2<kt_double> & rCoarseSearchOffset,
  const Vector2<kt_double> & rCoarseSearchResolution, kt_bool doPenalize,
  kt_bool doRefineMatch)
{
  SourceEstimate estimate;
  m_ScoreSource = source;
  const kt_double coarseAngleOffset = m_pMapper->m_pCoarseSearchAngleOffset->GetValue();
  const kt_double coarseAngleResolution = m_pMapper->m_pCoarseAngleResolution->GetValue();

  Matrix3 coarseCovariance;
  m_ComputeFullCovariance = true;
  estimate.response = CorrelateScan(pScan, rSearchCenter, rCoarseSearchOffset,
      rCoarseSearchResolution, coarseAngleOffset, coarseAngleResolution, doPenalize,
      estimate.mean, coarseCovariance, false);
  m_ComputeFullCovariance = false;
  const Matrix3 full = m_FullCovariance;
  estimate.valid = estimate.response > KT_TOLERANCE;
  if (!estimate.valid) {
    return estimate;
  }

  // Heading variance at a fixed position, from the fine pass (the coarse angular step is too
  // coarse for a sharp wall heading).
  kt_double conditionalHeadingVariance = full(2, 2);
  if (doRefineMatch) {
    Matrix3 fineCovariance = coarseCovariance;
    const Vector2<kt_double> fineSearchOffset(rCoarseSearchResolution * 0.5);
    const Vector2<kt_double> fineSearchResolution(m_pCorrelationGrid->GetResolution(),
      m_pCorrelationGrid->GetResolution());
    estimate.response = CorrelateScan(pScan, estimate.mean, fineSearchOffset,
        fineSearchResolution, 0.5 * coarseAngleResolution,
        m_pMapper->m_pFineSearchAngleOffset->GetValue(), doPenalize, estimate.mean,
        fineCovariance, true);
    conditionalHeadingVariance = fineCovariance(2, 2);
  }

  // Position block and position-heading coupling from the coarse surface, which sees the whole
  // window: flat directions, and the ridge of poses rotating about a single marker. Marginal
  // heading variance = conditional (fine) + the part the coupling explains; the Schur complement
  // is then the conditional variance, so the matrix stays positive definite.
  Matrix3 covariance = full;
  const kt_double a = full(0, 0);
  const kt_double b = full(0, 1);
  const kt_double d = full(1, 1);
  const kt_double det = a * d - b * b;
  const kt_double px = full(0, 2);
  const kt_double py = full(1, 2);
  // det is tiny for a sharp sequential match (floors of 2.5e-5 m^2): any positive value counts,
  // or the coupling stays without its share of heading variance and the matrix is indefinite.
  const kt_double explained = det > 0.0 ?
    (d * px * px - 2.0 * b * px * py + a * py * py) / det : 0.0;
  if (det <= 0.0) {
    covariance(0, 2) = covariance(2, 0) = covariance(1, 2) = covariance(2, 1) = 0.0;
  }
  covariance(2, 2) = math::Maximum(conditionalHeadingVariance, KT_TOLERANCE) + explained;
  estimate.covariance = PositiveDefinite(covariance);
  return estimate;
}

kt_double ScanMatcher::MatchInformation(
  LocalizedRangeScan * pScan, const Pose2 & rSearchCenter,
  const Vector2<kt_double> & rCoarseSearchOffset,
  const Vector2<kt_double> & rCoarseSearchResolution, kt_bool doPenalize,
  kt_bool doRefineMatch, Pose2 & rMean, Matrix3 & rCovariance)
{
  const SourceEstimate geometry = EstimateSource(ScoreSource::Geometry, pScan, rSearchCenter,
      rCoarseSearchOffset, rCoarseSearchResolution, doPenalize, doRefineMatch);
  if (!geometry.valid) {
    m_ScoreSource = ScoreSource::Combined;
    return -1.0;
  }
  // Without reflectors in the grid (the odometry prior also comes this way) there is nothing to
  // fuse with geometry.
  SourceEstimate reflector;
  if (m_ReflectorsActive) {
    reflector = EstimateSource(ScoreSource::Reflector, pScan, rSearchCenter,
        rCoarseSearchOffset, rCoarseSearchResolution, doPenalize, doRefineMatch);
  }
  m_ScoreSource = ScoreSource::Combined;
  if (reflector.valid && !m_pMapper->m_pReflectorFuseHeading->GetValue()) {
    // Position only: take the reflector pose at geometry's heading and drop its heading
    // information (and the coupling), so the fused heading is geometry's.
    reflector.mean = Pose2(reflector.mean.GetX(), reflector.mean.GetY(),
        geometry.mean.GetHeading());
    reflector.covariance(0, 2) = reflector.covariance(2, 0) = 0.0;
    reflector.covariance(1, 2) = reflector.covariance(2, 1) = 0.0;
    reflector.covariance(2, 2) = 1e6;
  }

  rMean = geometry.mean;
  rCovariance = geometry.covariance;
  // Determinant tolerance for these covariances: variances of 1e-6 m^2 and rad^2 give
  // determinants around 1e-18, far below InverseFast's default.
  const kt_double kInverseTolerance = 1e-30;
  kt_bool fused = false;
  kt_bool gated = false;
  kt_double d2 = 0.0;

  Matrix3 geometryInformation;
  Matrix3 reflectorInformation;
  if (reflector.valid && CountMatchedReflectors(pScan, reflector.mean) >= 1 &&
    geometry.covariance.InverseFast(geometryInformation, kInverseTolerance) &&
    reflector.covariance.InverseFast(reflectorInformation, kInverseTolerance))
  {
    const kt_double delta[3] = {reflector.mean.GetX() - geometry.mean.GetX(),
      reflector.mean.GetY() - geometry.mean.GetY(),
      math::NormalizeAngle(reflector.mean.GetHeading() - geometry.mean.GetHeading())};

    // Consistency gate: d^2 = delta' (Sg + Sr)^-1 delta.
    Matrix3 joint = geometry.covariance;
    joint += reflector.covariance;
    Matrix3 jointInverse;
    if (joint.InverseFast(jointInverse, kInverseTolerance)) {
      for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
          d2 += delta[r] * jointInverse(r, c) * delta[c];
        }
      }
    }
    const kt_double chi2 = m_pMapper->m_pReflectorConsistencyGateChi2->GetValue();
    gated = chi2 > 0.0 && d2 > chi2;

    // Fusion: S = (Sg^-1 + Sr^-1)^-1, T = Tg + S Sr^-1 delta.
    Matrix3 information = geometryInformation;
    information += reflectorInformation;
    Matrix3 fusedCovariance;
    if (!gated && information.InverseFast(fusedCovariance, kInverseTolerance)) {
      kt_double weighted[3] = {0.0, 0.0, 0.0};
      for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
          weighted[r] += reflectorInformation(r, c) * delta[c];
        }
      }
      kt_double step[3] = {0.0, 0.0, 0.0};
      for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
          step[r] += fusedCovariance(r, c) * weighted[c];
        }
      }
      rMean = Pose2(geometry.mean.GetX() + step[0], geometry.mean.GetY() + step[1],
          math::NormalizeAngle(geometry.mean.GetHeading() + step[2]));
      rCovariance = PositiveDefinite(fusedCovariance);
      fused = true;
    }
  }

  if (doPenalize) {
    m_LastReflectorStats.active = m_ReflectorsActive;
    m_LastReflectorStats.scanPoints = static_cast<kt_int32u>(pScan->GetReflectorPoints().size());
    m_LastReflectorStats.referencePoints = m_ReferenceReflectorCount;
    m_LastReflectorStats.matched = CountMatchedReflectors(pScan, rMean);
    m_LastReflectorStats.response = geometry.response;
    m_LastReflectorStats.fused = fused;
    m_LastReflectorStats.gated = gated;
    m_LastReflectorStats.gateD2 = d2;
  }
  return math::Minimum(geometry.response, 1.0);
}


void ScanMatcher::FinishSequentialMatch(
  LocalizedRangeScan * pScan, const Pose2 & rPredicted, Pose2 & rMean,
  const Matrix3 & rCovariance)
{
  m_LastMatch.valid = true;
  m_LastMatch.predicted = rPredicted;
  m_LastMatch.scanMean = rMean;
  m_LastMatch.scanCovariance = rCovariance;
  m_LastMatch.mean = rMean;
  m_LastMatch.poseCovariance = rCovariance;
  if (!m_pMapper->m_pOdometryPriorFusion->GetValue()) {
    return;
  }
  const LocalizedRangeScan * pLastScan =
    m_pMapper->m_pMapperSensorManager->GetLastScan(pScan->GetSensorName());
  if (pLastScan == NULL) {
    return;
  }

  // Prior from the odometry motion since the last scan: position and heading uncertainty of
  // the robot, growing with distance and turn.
  const Pose2 & rFrom = pLastScan->GetOdometricPose();
  const Pose2 & rTo = pScan->GetOdometricPose();
  const kt_double distance = (rTo.GetPosition() - rFrom.GetPosition()).Length();
  const kt_double turn = std::fabs(math::NormalizeAngle(rTo.GetHeading() - rFrom.GetHeading()));
  // Along the direction of travel odometry errs by its scale; across it by heading error times
  // distance, which a wall usually knows better.
  const kt_double sigmaAlong = m_pMapper->m_pOdometryPriorStddevXyPerMeter->GetValue() *
    distance + m_pMapper->m_pOdometryPriorStddevXyMin->GetValue();
  const kt_double sigmaCross = m_pMapper->m_pOdometryPriorStddevCrossPerMeter->GetValue() *
    distance + m_pMapper->m_pOdometryPriorStddevXyMin->GetValue();
  const kt_double sigmaYaw =
    m_pMapper->m_pOdometryPriorStddevYawPerRadian->GetValue() * turn +
    m_pMapper->m_pOdometryPriorStddevYawPerMeter->GetValue() * distance +
    m_pMapper->m_pOdometryPriorStddevYawMin->GetValue();
  // The match is of the sensor pose: a robot heading error also moves the sensor, by the
  // lever arm of its mounting offset.
  const Pose2 & rOffset = pScan->GetLaserRangeFinder()->GetOffsetPose();
  const kt_double varYaw = math::Square(sigmaYaw);
  // Position block in the map frame, from along/across the robot's predicted heading.
  const kt_double travelHeading = rPredicted.GetHeading() - rOffset.GetHeading();
  const kt_double ch = cos(travelHeading);
  const kt_double sh = sin(travelHeading);
  const kt_double varAlong = math::Square(sigmaAlong);
  const kt_double varCross = math::Square(sigmaCross);
  const kt_double pxx = ch * ch * varAlong + sh * sh * varCross;
  const kt_double pyy = sh * sh * varAlong + ch * ch * varCross;
  const kt_double pxy = ch * sh * (varAlong - varCross);
  Pose2 predicted = rPredicted;
  Matrix3 prior;
  if (m_pMapper->m_pOdometryPriorFuseHeading->GetValue()) {
    const kt_double robotHeading = rPredicted.GetHeading() - rOffset.GetHeading();
    const kt_double c = cos(robotHeading);
    const kt_double s = sin(robotHeading);
    const kt_double jx = -s * rOffset.GetX() - c * rOffset.GetY();
    const kt_double jy = c * rOffset.GetX() - s * rOffset.GetY();
    prior(0, 0) = pxx + jx * jx * varYaw;
    prior(1, 1) = pyy + jy * jy * varYaw;
    prior(0, 1) = prior(1, 0) = pxy + jx * jy * varYaw;
    prior(0, 2) = prior(2, 0) = jx * varYaw;
    prior(1, 2) = prior(2, 1) = jy * varYaw;
    prior(2, 2) = varYaw;
  } else {
    // Position only. Odometry's heading drifts (gyro bias, scrub) and a wall fixes heading far
    // better than the coarse-search floor of the scan's heading variance admits, so taking
    // heading from odometry walked the pose across a corridor (REFLECTOR_SLAM.md 5.10). The prior
    // is the odometry robot position with the sensor placed at the scan's heading.
    const kt_double predictedRobotHeading = rPredicted.GetHeading() - rOffset.GetHeading();
    const kt_double robotX = rPredicted.GetX() - (cos(predictedRobotHeading) * rOffset.GetX() -
      sin(predictedRobotHeading) * rOffset.GetY());
    const kt_double robotY = rPredicted.GetY() - (sin(predictedRobotHeading) * rOffset.GetX() +
      cos(predictedRobotHeading) * rOffset.GetY());
    const kt_double scanRobotHeading = rMean.GetHeading() - rOffset.GetHeading();
    predicted = Pose2(
      robotX + cos(scanRobotHeading) * rOffset.GetX() - sin(scanRobotHeading) * rOffset.GetY(),
      robotY + sin(scanRobotHeading) * rOffset.GetX() + cos(scanRobotHeading) * rOffset.GetY(),
      rMean.GetHeading());
    prior(0, 0) = pxx;
    prior(1, 1) = pyy;
    prior(0, 1) = prior(1, 0) = pxy;
    prior(2, 2) = 1e6;
  }

  const kt_double kInverseTolerance = 1e-30;
  const kt_double delta[3] = {rMean.GetX() - predicted.GetX(),
    rMean.GetY() - predicted.GetY(),
    math::NormalizeAngle(rMean.GetHeading() - predicted.GetHeading())};
  Matrix3 joint = rCovariance;
  joint += prior;
  Matrix3 jointInverse;
  Matrix3 scanInformation;
  Matrix3 priorInformation;
  if (!joint.InverseFast(jointInverse, kInverseTolerance) ||
    !rCovariance.InverseFast(scanInformation, kInverseTolerance) ||
    !prior.InverseFast(priorInformation, kInverseTolerance))
  {
    return;
  }
  kt_double d2 = 0.0;
  for (int r = 0; r < 3; r++) {
    for (int k = 0; k < 3; k++) {
      d2 += delta[r] * jointInverse(r, k) * delta[k];
    }
  }
  m_LastMatch.priorD2 = d2;
  // A scan far from odometry is more likely right (wheel slip) than a blind direction: keep it.
  const kt_double gate = m_pMapper->m_pOdometryPriorGateChi2->GetValue();
  if (gate > 0.0 && d2 > gate) {
    m_LastMatch.priorGated = true;
    return;
  }

  // Fusion: S = (Ss^-1 + So^-1)^-1, T = To + S Ss^-1 (Ts - To).
  Matrix3 information = scanInformation;
  information += priorInformation;
  Matrix3 fused;
  if (!information.InverseFast(fused, kInverseTolerance)) {
    return;
  }
  kt_double weighted[3] = {0.0, 0.0, 0.0};
  for (int r = 0; r < 3; r++) {
    for (int k = 0; k < 3; k++) {
      weighted[r] += scanInformation(r, k) * delta[k];
    }
  }
  kt_double step[3] = {0.0, 0.0, 0.0};
  for (int r = 0; r < 3; r++) {
    for (int k = 0; k < 3; k++) {
      step[r] += fused(r, k) * weighted[k];
    }
  }
  rMean = Pose2(predicted.GetX() + step[0], predicted.GetY() + step[1],
      math::NormalizeAngle(predicted.GetHeading() + step[2]));
  m_LastMatch.prior = true;
  m_LastMatch.mean = rMean;
  m_LastMatch.poseCovariance = PositiveDefinite(fused);
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
class BreadthFirstTraversal : public GraphTraversal<T>
{
public:
  /**
   * Constructs a breadth-first traverser for the given graph
   */
  BreadthFirstTraversal()
  {
  }
  explicit BreadthFirstTraversal(Graph<T> * pGraph)
  : GraphTraversal<T>(pGraph)
  {
  }

  /**
   * Destructor
   */
  virtual ~BreadthFirstTraversal()
  {
  }

public:
  /**
   * Traverse the graph starting with the given vertex; applies the visitor to visited nodes
   * @param pStartVertex
   * @param pVisitor
   * @return visited vertice scans
   */
  virtual std::vector<T *> TraverseForScans(Vertex<T> * pStartVertex, Visitor<T> * pVisitor)
  {
    std::vector<Vertex<T> *> validVertices = TraverseForVertices(pStartVertex, pVisitor);

    std::vector<T *> objects;
    forEach(typename std::vector<Vertex<T> *>, &validVertices)
    {
      objects.push_back((*iter)->GetObject());
    }

    return objects;
  }

  /**
   * Traverse the graph starting with the given vertex; applies the visitor to visited nodes
   * @param pStartVertex
   * @param pVisitor
   * @return visited vertices
   */
  virtual std::vector<Vertex<T> *> TraverseForVertices(
    Vertex<T> * pStartVertex,
    Visitor<T> * pVisitor)
  {
    std::queue<Vertex<T> *> toVisit;
    std::set<Vertex<T> *> seenVertices;
    std::vector<Vertex<T> *> validVertices;

    toVisit.push(pStartVertex);
    seenVertices.insert(pStartVertex);

    do {
      Vertex<T> * pNext = toVisit.front();
      toVisit.pop();

      if (pNext != NULL && pVisitor->Visit(pNext)) {
        // vertex is valid, explore neighbors
        validVertices.push_back(pNext);

        std::vector<Vertex<T> *> adjacentVertices = pNext->GetAdjacentVertices();
        forEach(typename std::vector<Vertex<T> *>, &adjacentVertices)
        {
          Vertex<T> * pAdjacent = *iter;

          // adjacent vertex has not yet been seen, add to queue for processing
          if (seenVertices.find(pAdjacent) == seenVertices.end()) {
            toVisit.push(pAdjacent);
            seenVertices.insert(pAdjacent);
          }
        }
      }
    } while (toVisit.empty() == false);

    return validVertices;
  }

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GraphTraversal<T>);
  }
};    // class BreadthFirstTraversal

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

class NearScanVisitor : public Visitor<LocalizedRangeScan>
{
public:
  NearScanVisitor(LocalizedRangeScan * pScan, kt_double maxDistance, kt_bool useScanBarycenter)
  : m_MaxDistanceSquared(math::Square(maxDistance)),
    m_UseScanBarycenter(useScanBarycenter)
  {
    m_CenterPose = pScan->GetReferencePose(m_UseScanBarycenter);
  }

  virtual kt_bool Visit(Vertex<LocalizedRangeScan> * pVertex)
  {
    try {
      LocalizedRangeScan * pScan = pVertex->GetObject();
      Pose2 pose = pScan->GetReferencePose(m_UseScanBarycenter);
      kt_double squaredDistance = pose.GetPosition().SquaredDistance(m_CenterPose.GetPosition());
      return squaredDistance <= m_MaxDistanceSquared - KT_TOLERANCE;
    } catch (...) {
      // relocalization vertex elements missing
      std::cout << "Unable to visit valid vertex elements!" << std::endl;
      return false;
    }
  }

protected:
  Pose2 m_CenterPose;
  kt_double m_MaxDistanceSquared;
  kt_bool m_UseScanBarycenter;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Visitor<LocalizedRangeScan>);
    ar & BOOST_SERIALIZATION_NVP(m_CenterPose);
    ar & BOOST_SERIALIZATION_NVP(m_MaxDistanceSquared);
    ar & BOOST_SERIALIZATION_NVP(m_UseScanBarycenter);
  }
};    // NearScanVisitor

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

class NearPoseVisitor : public Visitor<LocalizedRangeScan>
{
public:
  NearPoseVisitor(Pose2 refPose, kt_double maxDistance, kt_bool useScanBarycenter)
  : m_MaxDistanceSquared(math::Square(maxDistance)),
    m_UseScanBarycenter(useScanBarycenter)
  {
    m_CenterPose = refPose;
  }

  virtual kt_bool Visit(Vertex<LocalizedRangeScan> * pVertex)
  {
    LocalizedRangeScan * pScan = pVertex->GetObject();

    Pose2 pose = pScan->GetReferencePose(m_UseScanBarycenter);

    kt_double squaredDistance = pose.GetPosition().SquaredDistance(m_CenterPose.GetPosition());
    return squaredDistance <= m_MaxDistanceSquared - KT_TOLERANCE;
  }

protected:
  Pose2 m_CenterPose;
  kt_double m_MaxDistanceSquared;
  kt_bool m_UseScanBarycenter;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Visitor<LocalizedRangeScan>);
    ar & BOOST_SERIALIZATION_NVP(m_CenterPose);
    ar & BOOST_SERIALIZATION_NVP(m_MaxDistanceSquared);
    ar & BOOST_SERIALIZATION_NVP(m_UseScanBarycenter);
  }
};    // NearPoseVisitor

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////


MapperGraph::MapperGraph(Mapper * pMapper, kt_double rangeThreshold)
: m_pMapper(pMapper)
{
  m_pLoopScanMatcher = ScanMatcher::Create(pMapper,
    m_pMapper->m_pLoopSearchSpaceDimension->GetValue(),
    m_pMapper->m_pLoopSearchSpaceResolution->GetValue(),
    m_pMapper->m_pLoopSearchSpaceSmearDeviation->GetValue(), rangeThreshold);
  assert(m_pLoopScanMatcher);

  m_pTraversal = new BreadthFirstTraversal<LocalizedRangeScan>(this);
}

MapperGraph::~MapperGraph()
{
  if (m_pLoopScanMatcher) {
    delete m_pLoopScanMatcher;
    m_pLoopScanMatcher = NULL;
  }
  if (m_pTraversal) {
    delete m_pTraversal;
    m_pTraversal = NULL;
  }
}

Vertex<LocalizedRangeScan> * MapperGraph::AddVertex(LocalizedRangeScan * pScan)
{
  assert(pScan);

  if (pScan != NULL) {
    Vertex<LocalizedRangeScan> * pVertex = new Vertex<LocalizedRangeScan>(pScan);
    Graph<LocalizedRangeScan>::AddVertex(pScan->GetSensorName(), pVertex);
    if (m_pMapper->m_pScanOptimizer != NULL) {
      m_pMapper->m_pScanOptimizer->AddNode(pVertex);
    }
    return pVertex;
  }

  return nullptr;
}

void MapperGraph::AddEdges(LocalizedRangeScan * pScan, const Matrix3 & rCovariance)
{
  m_LinkOrigin = LinkInfo::kOriginSequential;
  MapperSensorManager * pSensorManager = m_pMapper->m_pMapperSensorManager;

  const Name rSensorName = pScan->GetSensorName();


  // link to previous scan
  kt_int32s previousScanNum = pScan->GetStateId() - 1;
  if (pSensorManager->GetLastScan(rSensorName) != NULL) {
    assert(previousScanNum >= 0);
    LocalizedRangeScan * pPrevScan = pSensorManager->GetScan(rSensorName, previousScanNum);
    if (!pPrevScan) {
      return;
    }
    LinkScans(pPrevScan, pScan, pScan->GetSensorPose(), rCovariance);
  }

  Pose2Vector means;
  std::vector<Matrix3> covariances;

  // first scan (link to first scan of other robots)
  if (pSensorManager->GetLastScan(rSensorName) == NULL) {
    assert(pSensorManager->GetScans(rSensorName).size() == 1);

    std::vector<Name> deviceNames = pSensorManager->GetSensorNames();
    forEach(std::vector<Name>, &deviceNames)
    {
      const Name & rCandidateSensorName = *iter;

      // skip if candidate device is the same or other device has no scans
      if ((rCandidateSensorName == rSensorName) ||
        (pSensorManager->GetScans(rCandidateSensorName).empty()))
      {
        continue;
      }

      Pose2 bestPose;
      Matrix3 covariance;
      kt_double response = m_pMapper->m_pSequentialScanMatcher->MatchScan<LocalizedRangeScanMap>(
        pScan,
        pSensorManager->GetScans(rCandidateSensorName),
        bestPose, covariance);
      LinkScans(pSensorManager->GetScan(rCandidateSensorName, 0), pScan, bestPose, covariance);

      // only add to means and covariances if response was high "enough"
      if (response > m_pMapper->m_pLinkMatchMinimumResponseFine->GetValue()) {
        means.push_back(bestPose);
        covariances.push_back(covariance);
      }
    }
  } else {
    // link to running scans
    Pose2 scanPose = pScan->GetSensorPose();
    means.push_back(scanPose);
    // With the odometry prior (REFLECTOR_SLAM.md 5.10) the pose also carries odometry's
    // information: weigh it against near chains by the fused covariance. The edges keep the
    // fused pose: the scan-only one sits on the search window's edge along a corridor, and as an
    // edge it outvoted the odometry edges and brought the start-up lag back (§7.2).
    const ScanMatcher::MatchDiagnostics & rMatch =
      m_pMapper->m_pSequentialScanMatcher->GetLastMatch();
    covariances.push_back(rMatch.valid && rMatch.prior ? rMatch.poseCovariance : rCovariance);
    LinkChainToScan(pSensorManager->GetRunningScans(rSensorName), pScan, scanPose, rCovariance);
  }

  // link to other near chains (chains that include new scan are invalid)
  LinkNearChains(pScan, means, covariances);

  if (!means.empty()) {
    pScan->SetSensorPose(ComputeWeightedMean(means, covariances));
  }

  if (m_pMapper->m_pEnablePeriodicPoseGraphSolve->GetValue()) {
    ++m_NodesSinceLastPeriodicSolve;
    if (m_NodesSinceLastPeriodicSolve >=
      static_cast<int>(m_pMapper->m_pPeriodicSolveIntervalNodes->GetValue()))
    {
      m_NodesSinceLastPeriodicSolve = 0;
      CorrectPoses();
    }
  }
}

kt_bool MapperGraph::ClosureConsistentWithGraph(
  const LocalizedRangeScanVector & rChain, LocalizedRangeScan * pScan, const Pose2 & rClosurePose,
  const Matrix3 & rCovariance)
{
  const kt_double chi2 = m_pMapper->m_pLoopClosureGateChi2->GetValue();
  if (chi2 <= 0.0) {
    return true;
  }
  // L4 (REFLECTOR_SLAM.md §5.9): the correction the closure demands, delta = closure pose -
  // current pose of the scan (the chain is the reference), against the closure's covariance
  // plus the drift the graph may have accumulated over the path from the chain to the scan.
  const Pose2 current = pScan->GetSensorPose();
  const kt_double delta[3] = {rClosurePose.GetX() - current.GetX(),
    rClosurePose.GetY() - current.GetY(),
    math::NormalizeAngle(rClosurePose.GetHeading() - current.GetHeading())};

  const LocalizedRangeScan * pClosest = GetClosestScanToPose(rChain, rClosurePose);
  kt_double pathLength = 0.0;
  if (pClosest != nullptr) {
    MapperSensorManager * pSensorManager = m_pMapper->m_pMapperSensorManager;
    const kt_int32s from = math::Minimum(pClosest->GetUniqueId(), pScan->GetUniqueId());
    const kt_int32s to = math::Maximum(pClosest->GetUniqueId(), pScan->GetUniqueId());
    const LocalizedRangeScan * pPrevious = nullptr;
    for (kt_int32s id = from; id <= to; ++id) {
      const LocalizedRangeScan * pNext = id == pScan->GetUniqueId() ? pScan :
        pSensorManager->GetScan(id);
      if (pNext == nullptr) {
        continue;
      }
      if (pPrevious != nullptr) {
        pathLength += pPrevious->GetCorrectedPose().GetPosition().Distance(
          pNext->GetCorrectedPose().GetPosition());
      }
      pPrevious = pNext;
    }
  }
  Matrix3 joint = rCovariance;
  const kt_double driftXY = m_pMapper->m_pLoopClosureGateDriftXY->GetValue() * pathLength;
  const kt_double driftYaw = m_pMapper->m_pLoopClosureGateDriftYaw->GetValue() * pathLength;
  joint(0, 0) += driftXY * driftXY;
  joint(1, 1) += driftXY * driftXY;
  joint(2, 2) += driftYaw * driftYaw;
  Matrix3 information;
  if (!joint.InverseFast(information, 1e-30)) {
    return true;
  }
  kt_double d2 = 0.0;
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      d2 += delta[r] * information(r, c) * delta[c];
    }
  }
  return d2 <= chi2;
}

kt_bool MapperGraph::MatchChainGated(
  LocalizedRangeScan * pScan, const LocalizedRangeScanVector & rChain,
  kt_double minimumCoarseResponse, kt_double minimumFineResponse, LoopClosureStats & rStats,
  Pose2 & rMean, Matrix3 & rCovariance)
{
  // Coarse: the loop matcher's wide search (with the reflector layer and the ridge-aware
  // covariance when enabled, §5.9 L1/L5), gated on response and variance.
  Matrix3 covariance;
  const kt_double coarseResponse = m_pLoopScanMatcher->MatchScan(pScan, rChain, rMean,
      covariance, false, false);
  const Matrix3 coarseCovariance = covariance;
  ++rStats.attempts;
  rStats.coarseResponses.push_back(coarseResponse);
  if (m_pLoopScanMatcher->LastMatchUsedReflectors()) {
    ++rStats.reflectorsActive;
  }

  std::stringstream stream;
  stream << "COARSE RESPONSE: " << coarseResponse <<
    " (> " << minimumCoarseResponse << ")" << std::endl;
  stream << "            var: " << covariance(0, 0) << ",  " << covariance(1, 1) <<
    " (< " << m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue() << ")";
  m_pMapper->FireLoopClosureCheck(stream.str());

  if (coarseResponse <= minimumCoarseResponse) {
    ++rStats.rejectedCoarseResponse;
    return false;
  }
  if (covariance(0, 0) >= m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue() ||
    covariance(1, 1) >= m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue())
  {
    ++rStats.rejectedVariance;
    return false;
  }

  // Fine: the sequential matcher around the coarse pose, on a copy of the scan placed there.
  LocalizedRangeScan tmpScan(pScan->GetSensorName(), pScan->GetRangeReadingsVector());
  tmpScan.SetUniqueId(pScan->GetUniqueId());
  tmpScan.SetTime(pScan->GetTime());
  tmpScan.SetStateId(pScan->GetStateId());
  tmpScan.SetCorrectedPose(pScan->GetCorrectedPose());
  // The copy is made from the ranges only; carry the reflector data over when matching with
  // reflectors (REFLECTOR_SLAM.md §5.9, L5).
  if (m_pMapper->m_pReflectorsInLoopClosure->GetValue()) {
    tmpScan.SetReflectorWeights(pScan->GetReflectorWeights());
  }
  tmpScan.SetSensorPose(rMean);    // This also updates OdometricPose.
  const kt_double fineResponse = m_pMapper->m_pSequentialScanMatcher->MatchScan(&tmpScan,
      rChain, rMean, covariance, false);

  std::stringstream stream1;
  stream1 << "FINE RESPONSE: " << fineResponse << " (>" << minimumFineResponse << ")" <<
    std::endl;
  m_pMapper->FireLoopClosureCheck(stream1.str());

  // L2: the fine match only sees +-half a coarse step, so its covariance cannot show the coarse
  // search's ambiguity (a ridge along a corridor); keep both.
  if (m_pMapper->m_pLoopClosureCoarseCovariance->GetValue()) {
    covariance += coarseCovariance;
  }
  rCovariance = PositiveDefinite(covariance);

  if (fineResponse < minimumFineResponse) {
    ++rStats.rejectedFineResponse;
    m_pMapper->FireLoopClosureCheck("REJECTED!");
    return false;
  }
  if (!ClosureConsistentWithGraph(rChain, pScan, rMean, rCovariance)) {
    ++rStats.rejectedGate;
    m_pMapper->FireLoopClosureCheck("REJECTED by the consistency gate");
    return false;
  }
  ++rStats.accepted;
  return true;
}

kt_bool MapperGraph::TryCloseLoop(LocalizedRangeScan * pScan, const Name & rSensorName)
{
  kt_bool loopClosed = false;

  kt_int32u scanIndex = 0;

  LocalizedRangeScanVector candidateChain = FindPossibleLoopClosure(pScan, rSensorName, scanIndex);

  while (!candidateChain.empty()) {
    Pose2 bestPose;
    Matrix3 covariance;
    if (MatchChainGated(pScan, candidateChain,
      m_pMapper->m_pLoopMatchMinimumResponseCoarse->GetValue(),
      m_pMapper->m_pLoopMatchMinimumResponseFine->GetValue(), m_LoopClosureStats, bestPose,
      covariance))
    {
      m_pMapper->FireBeginLoopClosure("Closing loop...");

      // Stock Karto moves the scan to the closure pose before the solve, so the online pose
      // jumps by the full correction however uncertain the closure; off, the solve decides.
      if (m_pMapper->m_pLoopClosureMoveScan->GetValue()) {
        pScan->SetSensorPose(bestPose);
      }
      m_LinkOrigin = LinkInfo::kOriginLoopClosure;
      LinkChainToScan(candidateChain, pScan, bestPose, covariance);
      m_LinkOrigin = LinkInfo::kOriginSequential;
      CorrectPoses();

      m_pMapper->FireEndLoopClosure("Loop closed!");

      loopClosed = true;
    }

    candidateChain = FindPossibleLoopClosure(pScan, rSensorName, scanIndex);
  }

  return loopClosed;
}

LocalizedRangeScan * MapperGraph::GetClosestScanToPose(
  const LocalizedRangeScanVector & rScans,
  const Pose2 & rPose) const
{
  LocalizedRangeScan * pClosestScan = NULL;
  kt_double bestSquaredDistance = DBL_MAX;

  const_forEach(LocalizedRangeScanVector, &rScans)
  {
    Pose2 scanPose = (*iter)->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

    kt_double squaredDistance = rPose.GetPosition().SquaredDistance(scanPose.GetPosition());
    if (squaredDistance < bestSquaredDistance) {
      bestSquaredDistance = squaredDistance;
      pClosestScan = *iter;
    }
  }

  return pClosestScan;
}

Edge<LocalizedRangeScan> * MapperGraph::AddEdge(
  LocalizedRangeScan * pSourceScan,
  LocalizedRangeScan * pTargetScan, kt_bool & rIsNewEdge)
{
  std::map<int,
    Vertex<LocalizedRangeScan> *>::iterator v1 = m_Vertices[pSourceScan->GetSensorName()].find(
    pSourceScan->GetStateId());
  std::map<int,
    Vertex<LocalizedRangeScan> *>::iterator v2 = m_Vertices[pTargetScan->GetSensorName()].find(
    pTargetScan->GetStateId());

  if (v1 == m_Vertices[pSourceScan->GetSensorName()].end() ||
    v2 == m_Vertices[pSourceScan->GetSensorName()].end())
  {
    std::cout << "AddEdge: At least one vertex is invalid." << std::endl;
    return NULL;
  }

  // see if edge already exists
  const_forEach(std::vector<Edge<LocalizedRangeScan> *>, &(v1->second->GetEdges()))
  {
    Edge<LocalizedRangeScan> * pEdge = *iter;

    if (pEdge->GetTarget() == v2->second) {
      rIsNewEdge = false;
      return pEdge;
    }
  }

  Edge<LocalizedRangeScan> * pEdge = new Edge<LocalizedRangeScan>(v1->second, v2->second);
  Graph<LocalizedRangeScan>::AddEdge(pEdge);
  rIsNewEdge = true;
  return pEdge;
}

void MapperGraph::LinkScans(
  LocalizedRangeScan * pFromScan, LocalizedRangeScan * pToScan,
  const Pose2 & rMean, const Matrix3 & rCovariance)
{
  kt_bool isNewEdge = true;
  Edge<LocalizedRangeScan> * pEdge = AddEdge(pFromScan, pToScan, isNewEdge);

  if (pEdge == NULL) {
    return;
  }

  // only attach link information if the edge is new
  if (isNewEdge == true) {
    LinkInfo * pLinkInfo = new LinkInfo(
      pFromScan->GetCorrectedPose(), pToScan->GetCorrectedAt(rMean), rCovariance);
    pLinkInfo->SetOrigin(m_LinkOrigin);
    // also record the raw sensor-frame measurement (before the current, possibly wrong,
    // lidar<->base_link offset was applied/removed) so it can be used for online joint
    // extrinsic calibration; rMean is already the matcher's sensor-frame output for pToScan.
    pLinkInfo->UpdateSensorFrame(pFromScan->GetSensorPose(), rMean, rCovariance);
    // and the independent odometry-frame measurement (fused wheel+IMU odometry via TF, not
    // scan matching) -- this is what makes the extrinsic actually identifiable; see
    // UpdateOdometryFrame's doc comment and the PLAN.md addendum.
    //
    // Sequential edges only. odometry_edge_stddev_{xy,yaw} is a single fixed per-edge noise
    // model, and it can only be sized for one kind of span: consecutive nodes, which are
    // bounded apart by minimum_travel_distance/minimum_travel_heading/minimum_time_interval.
    // Near-chain and loop-closure edges join scans separated by an arbitrarily long driven
    // path, over which dead reckoning has drifted by far more than that stddev -- applying it
    // there is wildly overconfident, makes stale odometry fight the loop closure, and biases
    // the extrinsic. Sizing the stddev for those long edges instead (the previous approach)
    // just throws away the sequential edges' information, which is where the calibration
    // signal actually is.
    const kt_int32s stateIdDelta = pToScan->GetStateId() - pFromScan->GetStateId();
    if (pFromScan->GetSensorName() == pToScan->GetSensorName() &&
      (stateIdDelta == 1 || stateIdDelta == -1))
    {
      pLinkInfo->UpdateOdometryFrame(pFromScan->GetOdometricPose(), pToScan->GetOdometricPose());
    }
    pEdge->SetLabel(pLinkInfo);
    if (m_pMapper->m_pScanOptimizer != NULL) {
      m_pMapper->m_pScanOptimizer->AddConstraint(pEdge);
    }
  }
}

void MapperGraph::LinkNearChains(
  LocalizedRangeScan * pScan, Pose2Vector & rMeans,
  std::vector<Matrix3> & rCovariances)
{
  m_LinkOrigin = LinkInfo::kOriginNearChain;
  const std::vector<LocalizedRangeScanVector> nearChains = FindNearChains(pScan);
  const_forEach(std::vector<LocalizedRangeScanVector>, &nearChains)
  {
    if (iter->size() < m_pMapper->m_pLoopMatchMinimumChainSize->GetValue()) {
      continue;
    }

    Pose2 mean;
    Matrix3 covariance;
    kt_bool link = false;
    if (m_pMapper->m_pNearChainGated->GetValue()) {
      // L6 (REFLECTOR_SLAM.md §5.9): the sequential matcher only searches +-half its window
      // around the current, possibly drifted pose, so in a featureless corridor it confirms
      // the drift and states it as tight. Match near chains like loop closures instead.
      link = MatchChainGated(pScan, *iter,
          m_pMapper->m_pNearChainMinimumResponseCoarse->GetValue(),
          m_pMapper->m_pLinkMatchMinimumResponseFine->GetValue() - KT_TOLERANCE,
          m_NearChainStats, mean, covariance);
    } else {
      // match scan against "near" chain
      kt_double response = m_pMapper->m_pSequentialScanMatcher->MatchScan(pScan, *iter, mean,
          covariance, false);
      link = response > m_pMapper->m_pLinkMatchMinimumResponseFine->GetValue() - KT_TOLERANCE;
    }
    if (link) {
      rMeans.push_back(mean);
      rCovariances.push_back(covariance);
      LinkChainToScan(*iter, pScan, mean, covariance);
    }
  }
}

void MapperGraph::LinkChainToScan(
  const LocalizedRangeScanVector & rChain, LocalizedRangeScan * pScan,
  const Pose2 & rMean, const Matrix3 & rCovariance)
{
  Pose2 pose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

  const kt_int32u chainScans = m_pMapper->m_pLinkChainScans->GetValue();
  if (chainScans > 1) {
    LinkChainToScanSpread(rChain, pScan, pose, rMean, rCovariance, chainScans);
    return;
  }

  LocalizedRangeScan * pClosestScan = GetClosestScanToPose(rChain, pose);
  assert(pClosestScan != NULL);

  Pose2 closestScanPose =
    pClosestScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

  kt_double squaredDistance = pose.GetPosition().SquaredDistance(closestScanPose.GetPosition());
  if (squaredDistance <
    math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
  {
    LinkScans(pClosestScan, pScan, rMean, rCovariance);
  }
}

void MapperGraph::LinkChainToScanSpread(
  const LocalizedRangeScanVector & rChain, LocalizedRangeScan * pScan, const Pose2 & rPose,
  const Pose2 & rMean, const Matrix3 & rCovariance, kt_int32u chainScans)
{
  // L9 (REFLECTOR_SLAM.md §5.9): the match places the scan against the chain as a whole, so
  // link it to the chain's closest scans, not just one: a scan misplaced within a bent chain
  // then carries 1/n of the match instead of all of it.
  const kt_double maximumSquaredDistance =
    math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE;
  const kt_bool barycenter = m_pMapper->m_pUseScanBarycenter->GetValue();

  // Scans already linked to this one: an edge between two scans exists once, and LinkScans
  // would drop a second measurement (e.g. the previous scan's sequential edge).
  std::set<const LocalizedRangeScan *> linked;
  const auto vertex = m_Vertices[pScan->GetSensorName()].find(pScan->GetStateId());
  if (vertex != m_Vertices[pScan->GetSensorName()].end()) {
    for (const Edge<LocalizedRangeScan> * pEdge : vertex->second->GetEdges()) {
      linked.insert(pEdge->GetSource()->GetObject());
      linked.insert(pEdge->GetTarget()->GetObject());
    }
  }

  // As the single link: if the closest scan is already linked (the running buffer's previous
  // scan, by its sequential edge), the match is represented and nothing is added. Spreading
  // past it added the same match again on top of that edge, which outweighed odometry in the
  // solves along corridors without markers.
  LocalizedRangeScan * pClosest = GetClosestScanToPose(rChain, rPose);
  if (pClosest == NULL || linked.count(pClosest) > 0) {
    return;
  }

  std::vector<std::pair<kt_double, LocalizedRangeScan *>> candidates;
  for (LocalizedRangeScan * pCandidate : rChain) {
    if (pCandidate == pScan || linked.count(pCandidate) > 0) {
      continue;
    }
    const kt_double squaredDistance = rPose.GetPosition().SquaredDistance(
      pCandidate->GetReferencePose(barycenter).GetPosition());
    if (squaredDistance < maximumSquaredDistance) {
      candidates.emplace_back(squaredDistance, pCandidate);
    }
  }
  const size_t n = std::min<size_t>(chainScans, candidates.size());
  if (n == 0) {
    return;
  }
  std::partial_sort(candidates.begin(), candidates.begin() + n, candidates.end(),
    [](const auto & a, const auto & b) {return a.first < b.first;});

  // Each edge with 1/n of the match's information, so together they state it once, plus the
  // lever arm: the scan's position relative to a chain scan d metres away is also off by
  // heading error times d, perpendicular to d (as uncertain as the match's heading). Without
  // it the far edges were overconfident and bent the online pose on the loops (§7.2).
  const kt_double headingVariance = rCovariance(2, 2);
  for (size_t i = 0; i < n; i++) {
    LocalizedRangeScan * pFrom = candidates[i].second;
    const kt_double rx = rMean.GetX() - pFrom->GetSensorPose().GetX();
    const kt_double ry = rMean.GetY() - pFrom->GetSensorPose().GetY();
    Matrix3 covariance;
    for (int r = 0; r < 3; r++) {
      for (int k = 0; k < 3; k++) {
        covariance(r, k) = rCovariance(r, k) * static_cast<kt_double>(n);
      }
    }
    // + headingVariance * a a' with a = (-ry, rx, 1): position moves with the heading error
    covariance(2, 2) += headingVariance;
    covariance(0, 0) += headingVariance * ry * ry;
    covariance(1, 1) += headingVariance * rx * rx;
    covariance(0, 1) -= headingVariance * rx * ry;
    covariance(1, 0) -= headingVariance * rx * ry;
    covariance(0, 2) -= headingVariance * ry;
    covariance(2, 0) -= headingVariance * ry;
    covariance(1, 2) += headingVariance * rx;
    covariance(2, 1) += headingVariance * rx;
    LinkScans(pFrom, pScan, rMean, PositiveDefinite(covariance));
  }
}

std::vector<LocalizedRangeScanVector> MapperGraph::FindNearChains(LocalizedRangeScan * pScan)
{
  std::vector<LocalizedRangeScanVector> nearChains;

  Pose2 scanPose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

  // to keep track of which scans have been added to a chain
  LocalizedRangeScanVector processed;

  const LocalizedRangeScanVector nearLinkedScans = FindNearLinkedScans(pScan,
      m_pMapper->m_pLinkScanMaximumDistance->GetValue());
  const_forEach(LocalizedRangeScanVector, &nearLinkedScans)
  {
    LocalizedRangeScan * pNearScan = *iter;

    if (pNearScan == pScan) {
      continue;
    }

    // scan has already been processed, skip
    if (find(processed.begin(), processed.end(), pNearScan) != processed.end()) {
      continue;
    }

    processed.push_back(pNearScan);

    // build up chain
    kt_bool isValidChain = true;
    std::list<LocalizedRangeScan *> chain;

    // add scans before current scan being processed
    for (kt_int32s candidateScanNum = pNearScan->GetStateId() - 1; candidateScanNum >= 0;
      candidateScanNum--)
    {
      LocalizedRangeScan * pCandidateScan = m_pMapper->m_pMapperSensorManager->GetScan(
        pNearScan->GetSensorName(),
        candidateScanNum);

      // chain is invalid--contains scan being added
      if (pCandidateScan == pScan) {
        isValidChain = false;
      }

      // probably removed in localization mode
      if (pCandidateScan == NULL) {
        continue;
      }

      Pose2 candidatePose = pCandidateScan->GetReferencePose(
        m_pMapper->m_pUseScanBarycenter->GetValue());
      kt_double squaredDistance =
        scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());

      if (squaredDistance <
        math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
      {
        chain.push_front(pCandidateScan);
        processed.push_back(pCandidateScan);
      } else {
        break;
      }
    }

    chain.push_back(pNearScan);

    // add scans after current scan being processed
    kt_int32u end =
      static_cast<kt_int32u>(m_pMapper->m_pMapperSensorManager->GetScans(
        pNearScan->GetSensorName()).size());
    for (kt_int32u candidateScanNum = pNearScan->GetStateId() + 1; candidateScanNum < end;
      candidateScanNum++)
    {
      LocalizedRangeScan * pCandidateScan = m_pMapper->m_pMapperSensorManager->GetScan(
        pNearScan->GetSensorName(),
        candidateScanNum);

      if (pCandidateScan == pScan) {
        isValidChain = false;
      }

      // probably removed in localization mode
      if (pCandidateScan == NULL) {
        continue;
      }

      Pose2 candidatePose = pCandidateScan->GetReferencePose(
        m_pMapper->m_pUseScanBarycenter->GetValue());
      kt_double squaredDistance =
        scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());

      if (squaredDistance <
        math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
      {
        chain.push_back(pCandidateScan);
        processed.push_back(pCandidateScan);
      } else {
        break;
      }
    }

    if (isValidChain) {
      // change list to vector
      LocalizedRangeScanVector tempChain;
      std::copy(chain.begin(), chain.end(), std::inserter(tempChain, tempChain.begin()));
      // add chain to collection
      nearChains.push_back(tempChain);
    }
  }

  return nearChains;
}

LocalizedRangeScanVector MapperGraph::FindNearLinkedScans(
  LocalizedRangeScan * pScan,
  kt_double maxDistance)
{
  NearScanVisitor * pVisitor = new NearScanVisitor(pScan, maxDistance,
      m_pMapper->m_pUseScanBarycenter->GetValue());
  LocalizedRangeScanVector nearLinkedScans = m_pTraversal->TraverseForScans(GetVertex(
        pScan), pVisitor);
  delete pVisitor;

  return nearLinkedScans;
}

std::vector<Vertex<LocalizedRangeScan> *> MapperGraph::FindNearLinkedVertices(
  LocalizedRangeScan * pScan, kt_double maxDistance)
{
  NearScanVisitor * pVisitor = new NearScanVisitor(pScan, maxDistance,
      m_pMapper->m_pUseScanBarycenter->GetValue());
  std::vector<Vertex<LocalizedRangeScan> *> nearLinkedVertices =
    m_pTraversal->TraverseForVertices(GetVertex(
        pScan), pVisitor);
  delete pVisitor;

  return nearLinkedVertices;
}

LocalizedRangeScanVector MapperGraph::FindNearByScans(
  Name name, const Pose2 refPose,
  kt_double maxDistance)
{
  NearPoseVisitor * pVisitor = new NearPoseVisitor(refPose, maxDistance,
      m_pMapper->m_pUseScanBarycenter->GetValue());

  Vertex<LocalizedRangeScan> * closestVertex = FindNearByScan(name, refPose);

  LocalizedRangeScanVector nearLinkedScans =
    m_pTraversal->TraverseForScans(closestVertex, pVisitor);
  delete pVisitor;

  return nearLinkedScans;
}

std::vector<Vertex<LocalizedRangeScan> *> MapperGraph::FindNearByVertices(
  Name name,
  const Pose2 refPose,
  kt_double maxDistance)
{
  VertexMap vertexMap = GetVertices();
  std::map<int, Vertex<LocalizedRangeScan> *> & vertices = vertexMap[name];

  std::vector<Vertex<LocalizedRangeScan> *> vertices_to_search;
  std::map<int, Vertex<LocalizedRangeScan> *>::iterator it;
  for (it = vertices.begin(); it != vertices.end(); ++it) {
    if (it->second) {
      vertices_to_search.push_back(it->second);
    }
  }

  const size_t dim = 2;

  typedef VertexVectorPoseNanoFlannAdaptor<std::vector<Vertex<LocalizedRangeScan> *>> P2KD;
  const P2KD p2kd(vertices_to_search);

  typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, P2KD>, P2KD,
      dim> my_kd_tree_t;

  my_kd_tree_t index(dim, p2kd, nanoflann::KDTreeSingleIndexAdaptorParams(10) );
  index.buildIndex();

  std::vector<std::pair<size_t, double>> ret_matches;
  const double query_pt[2] = {refPose.GetX(), refPose.GetY()};
  nanoflann::SearchParams params;
  const size_t num_results = index.radiusSearch(&query_pt[0], maxDistance, ret_matches, params);

  std::vector<Vertex<LocalizedRangeScan> *> rtn_vertices;
  rtn_vertices.reserve(ret_matches.size());
  for (uint i = 0; i != ret_matches.size(); i++) {
    rtn_vertices.push_back(vertices_to_search[ret_matches[i].first]);
  }
  return rtn_vertices;
}

Vertex<LocalizedRangeScan> * MapperGraph::FindNearByScan(Name name, const Pose2 refPose)
{
  VertexMap vertexMap = GetVertices();
  std::map<int, Vertex<LocalizedRangeScan> *> & vertices = vertexMap[name];

  std::vector<Vertex<LocalizedRangeScan> *> vertices_to_search;
  std::map<int, Vertex<LocalizedRangeScan> *>::iterator it;
  for (it = vertices.begin(); it != vertices.end(); ++it) {
    if (it->second) {
      vertices_to_search.push_back(it->second);
    }
  }

  size_t num_results = 1;
  const size_t dim = 2;

  typedef VertexVectorPoseNanoFlannAdaptor<std::vector<Vertex<LocalizedRangeScan> *>> P2KD;
  const P2KD p2kd(vertices_to_search);

  typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, P2KD>, P2KD,
      dim> my_kd_tree_t;

  my_kd_tree_t index(dim, p2kd, nanoflann::KDTreeSingleIndexAdaptorParams(10) );
  index.buildIndex();

  std::vector<size_t> ret_index(num_results);
  std::vector<double> out_dist_sqr(num_results);
  const double query_pt[2] = {refPose.GetX(), refPose.GetY()};
  num_results = index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

  if (num_results > 0) {
    return vertices_to_search[ret_index[0]];
  } else {
    return NULL;
  }
}

Pose2 MapperGraph::ComputeWeightedMean(
  const Pose2Vector & rMeans,
  const std::vector<Matrix3> & rCovariances) const
{
  assert(rMeans.size() == rCovariances.size());

  // compute sum of inverses and create inverse list
  std::vector<Matrix3> inverses;
  inverses.reserve(rCovariances.size());

  Matrix3 sumOfInverses;
  const_forEach(std::vector<Matrix3>, &rCovariances)
  {
    Matrix3 inverse = iter->Inverse();
    inverses.push_back(inverse);

    sumOfInverses += inverse;
  }
  Matrix3 inverseOfSumOfInverses = sumOfInverses.Inverse();

  // compute weighted mean
  Pose2 accumulatedPose;
  kt_double thetaX = 0.0;
  kt_double thetaY = 0.0;

  // The weights are full 3x3 information matrices, so a mean's heading enters the weighted x
  // and y through the position-heading terms. Means on either side of +-pi differ by 2 pi in
  // heading and would shift the position by metres; put every heading within pi of the first.
  const kt_double referenceHeading = rMeans.front().GetHeading();
  Pose2Vector::const_iterator meansIter = rMeans.begin();
  const_forEach(std::vector<Matrix3>, &inverses)
  {
    Pose2 pose = *meansIter;
    if (pose.GetHeading() - referenceHeading > KT_PI) {
      pose.SetHeading(pose.GetHeading() - KT_2PI);
    } else if (pose.GetHeading() - referenceHeading < -KT_PI) {
      pose.SetHeading(pose.GetHeading() + KT_2PI);
    }
    kt_double angle = pose.GetHeading();
    thetaX += cos(angle);
    thetaY += sin(angle);

    Matrix3 weight = inverseOfSumOfInverses * (*iter);
    accumulatedPose += weight * pose;

    ++meansIter;
  }

  thetaX /= rMeans.size();
  thetaY /= rMeans.size();
  accumulatedPose.SetHeading(atan2(thetaY, thetaX));

  return accumulatedPose;
}

LocalizedRangeScanVector MapperGraph::FindPossibleLoopClosure(
  LocalizedRangeScan * pScan,
  const Name & rSensorName,
  kt_int32u & rStartNum)
{
  LocalizedRangeScanVector chain;    // return value

  Pose2 pose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

  // possible loop closure chain should not include close scans that have a
  // path of links to the scan of interest
  const LocalizedRangeScanVector nearLinkedScans =
    FindNearLinkedScans(pScan, m_pMapper->m_pLoopSearchMaximumDistance->GetValue());

  kt_int32u nScans =
    static_cast<kt_int32u>(m_pMapper->m_pMapperSensorManager->GetScans(rSensorName).size());
  for (; rStartNum < nScans; rStartNum++) {
    LocalizedRangeScan * pCandidateScan = m_pMapper->m_pMapperSensorManager->GetScan(rSensorName,
        rStartNum);

    if (pCandidateScan == NULL) {
      continue;
    }

    Pose2 candidateScanPose = pCandidateScan->GetReferencePose(
      m_pMapper->m_pUseScanBarycenter->GetValue());

    kt_double squaredDistance = candidateScanPose.GetPosition().SquaredDistance(pose.GetPosition());
    if (squaredDistance <
      math::Square(m_pMapper->m_pLoopSearchMaximumDistance->GetValue()) + KT_TOLERANCE)
    {
      // a linked scan cannot be in the chain
      if (find(nearLinkedScans.begin(), nearLinkedScans.end(),
        pCandidateScan) != nearLinkedScans.end())
      {
        chain.clear();
      } else {
        chain.push_back(pCandidateScan);
      }
    } else {
      // return chain if it is long "enough"
      if (chain.size() >= m_pMapper->m_pLoopMatchMinimumChainSize->GetValue()) {
        return chain;
      } else {
        chain.clear();
      }
    }
  }

  return chain;
}

void MapperGraph::CorrectPoses()
{
  // optimize scans!
  ScanSolver * pSolver = m_pMapper->m_pScanOptimizer;
  if (pSolver != NULL) {
    pSolver->Compute();

    const_forEach(ScanSolver::IdPoseVector, &pSolver->GetCorrections())
    {
      LocalizedRangeScan * scan = m_pMapper->m_pMapperSensorManager->GetScan(iter->first);
      if (scan == NULL) {
        continue;
      }
      scan->SetCorrectedPoseAndUpdate(iter->second);
    }

    m_pMapper->ApplySensorOffsetCorrections();

    pSolver->Clear();
  }
}

void Mapper::ApplySensorOffsetCorrections()
{
  ScanSolver * pSolver = m_pScanOptimizer;
  if (pSolver == NULL) {
    return;
  }
  // feed the jointly-optimized sensor extrinsic estimate (if any) back into karto: this
  // instantly and retroactively affects GetSensorPose()/GetSensorAt() for every scan of that
  // sensor, so mark them dirty to force cached point clouds to be rebuilt against it.
  for (const auto & sensorCorrection : pSolver->GetSensorOffsetCorrections()) {
    const Name sensorName(sensorCorrection.first);
    LaserRangeFinder * pLrf =
      SensorManager::GetInstance()->GetSensorByName<LaserRangeFinder>(sensorName);
    if (pLrf == NULL) {
      continue;
    }
    pLrf->SetOffsetPose(sensorCorrection.second);
    // no scans yet before the first one is processed
    if (m_pMapperSensorManager == NULL || !m_pMapperSensorManager->HasScanManager(sensorName)) {
      continue;
    }
    kt_bool dirty = true;
    for (auto & idScan : m_pMapperSensorManager->GetScans(sensorName)) {
      if (idScan.second) {
        idScan.second->SetIsDirty(dirty);
      }
    }
  }
}

void MapperGraph::UpdateLoopScanMatcher(kt_double rangeThreshold)
{
  if (m_pLoopScanMatcher) {
    delete m_pLoopScanMatcher;
  }
  m_pLoopScanMatcher = ScanMatcher::Create(m_pMapper,
    m_pMapper->m_pLoopSearchSpaceDimension->GetValue(),
    m_pMapper->m_pLoopSearchSpaceResolution->GetValue(),
    m_pMapper->m_pLoopSearchSpaceSmearDeviation->GetValue(), rangeThreshold);
  assert(m_pLoopScanMatcher);
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Default constructor
 */
Mapper::Mapper()
: Module("Mapper"),
  m_Initialized(false),
  m_Deserialized(false),
  m_pSequentialScanMatcher(NULL),
  m_pMapperSensorManager(NULL),
  m_pGraph(NULL),
  m_pScanOptimizer(NULL)
{
  InitializeParameters();
}

/**
 * Default constructor
 */
Mapper::Mapper(const std::string & rName)
: Module(rName),
  m_Initialized(false),
  m_Deserialized(false),
  m_pSequentialScanMatcher(NULL),
  m_pMapperSensorManager(NULL),
  m_pGraph(NULL),
  m_pScanOptimizer(NULL)
{
  InitializeParameters();
}

/**
 * Destructor
 */
Mapper::~Mapper()
{
  Reset();

  delete m_pMapperSensorManager;
}

void Mapper::InitializeParameters()
{
  m_pUseScanMatching = new Parameter<kt_bool>(
    "UseScanMatching",
    "When set to true, the mapper will use a scan matching algorithm. "
    "In most real-world situations this should be set to true so that the "
    "mapper algorithm can correct for noise and errors in odometry and "
    "scan data. In some simulator environments where the simulated scan "
    "and odometry data are very accurate, the scan matching algorithm can "
    "produce worse results. In those cases set this to false to improve "
    "results.",
    true,
    GetParameterManager());

  m_pUseScanBarycenter = new Parameter<kt_bool>(
    "UseScanBarycenter",
    "Use the barycenter of scan endpoints to define distances between "
    "scans.",
    true, GetParameterManager());

  m_pMinimumTimeInterval = new Parameter<kt_double>(
    "MinimumTimeInterval",
    "Sets the minimum time between scans. If a new scan's time stamp is "
    "longer than MinimumTimeInterval from the previously processed scan, "
    "the mapper will use the data from the new scan. Otherwise, it will "
    "discard the new scan if it also does not meet the minimum travel "
    "distance and heading requirements. For performance reasons, it is "
    "generally it is a good idea to only process scans if a reasonable "
    "amount of time has passed. This parameter is particularly useful "
    "when there is a need to process scans while the robot is stationary.",
    3600, GetParameterManager());

  m_pMinimumTravelDistance = new Parameter<kt_double>(
    "MinimumTravelDistance",
    "Sets the minimum travel between scans.  If a new scan's position is "
    "more than minimumTravelDistance from the previous scan, the mapper "
    "will use the data from the new scan. Otherwise, it will discard the "
    "new scan if it also does not meet the minimum change in heading "
    "requirement. For performance reasons, generally it is a good idea to "
    "only process scans if the robot has moved a reasonable amount.",
    0.2, GetParameterManager());

  m_pMinimumTravelHeading = new Parameter<kt_double>(
    "MinimumTravelHeading",
    "Sets the minimum heading change between scans. If a new scan's "
    "heading is more than MinimumTravelHeading from the previous scan, the "
    "mapper will use the data from the new scan.  Otherwise, it will "
    "discard the new scan if it also does not meet the minimum travel "
    "distance requirement. For performance reasons, generally it is a good "
    "idea to only process scans if the robot has moved a reasonable "
    "amount.",
    math::DegreesToRadians(10), GetParameterManager());

  m_pScanBufferSize = new Parameter<kt_int32u>(
    "ScanBufferSize",
    "Scan buffer size is the length of the scan chain stored for scan "
    "matching. \"ScanBufferSize\" should be set to approximately "
    "\"ScanBufferMaximumScanDistance\" / \"MinimumTravelDistance\". The "
    "idea is to get an area approximately 20 meters long for scan "
    "matching. For example, if we add scans every MinimumTravelDistance == "
    "0.3 meters, then \"scanBufferSize\" should be 20 / 0.3 = 67.)",
    70, GetParameterManager());

  m_pScanBufferMaximumScanDistance = new Parameter<kt_double>(
    "ScanBufferMaximumScanDistance",
    "Scan buffer maximum scan distance is the maximum distance between the "
    "first and last scans in the scan chain stored for matching.",
    20.0, GetParameterManager());

  m_pLinkMatchMinimumResponseFine = new Parameter<kt_double>(
    "LinkMatchMinimumResponseFine",
    "Scans are linked only if the correlation response value is greater "
    "than this value.",
    0.8, GetParameterManager());

  m_pLinkScanMaximumDistance = new Parameter<kt_double>(
    "LinkScanMaximumDistance",
    "Maximum distance between linked scans.  Scans that are farther apart "
    "will not be linked regardless of the correlation response value.",
    10.0, GetParameterManager());

  m_pLoopSearchMaximumDistance = new Parameter<kt_double>(
    "LoopSearchMaximumDistance",
    "Scans less than this distance from the current position will be "
    "considered for a match in loop closure.",
    4.0, GetParameterManager());

  m_pDoLoopClosing = new Parameter<kt_bool>(
    "DoLoopClosing",
    "Enable/disable loop closure.",
    true, GetParameterManager());

  m_pLoopMatchMinimumChainSize = new Parameter<kt_int32u>(
    "LoopMatchMinimumChainSize",
    "When the loop closure detection finds a candidate it must be part of "
    "a large set of linked scans. If the chain of scans is less than this "
    "value we do not attempt to close the loop.",
    10, GetParameterManager());

  m_pEnablePeriodicPoseGraphSolve = new Parameter<kt_bool>(
    "EnablePeriodicPoseGraphSolve",
    "Enable/disable periodically re-solving the whole pose graph every "
    "PeriodicSolveIntervalNodes nodes, independent of loop closure.",
    false, GetParameterManager());

  m_pPeriodicSolveIntervalNodes = new Parameter<kt_int32u>(
    "PeriodicSolveIntervalNodes",
    "Number of nodes added between periodic pose graph solves, when enabled.",
    20, GetParameterManager());

  m_pLoopMatchMaximumVarianceCoarse = new Parameter<kt_double>(
    "LoopMatchMaximumVarianceCoarse",
    "The co-variance values for a possible loop closure have to be less "
    "than this value to consider a viable solution. This applies to the "
    "coarse search.",
    math::Square(0.4), GetParameterManager());

  m_pLoopMatchMinimumResponseCoarse = new Parameter<kt_double>(
    "LoopMatchMinimumResponseCoarse",
    "If response is larger then this, then initiate loop closure search at "
    "the coarse resolution.",
    0.8, GetParameterManager());

  m_pLoopMatchMinimumResponseFine = new Parameter<kt_double>(
    "LoopMatchMinimumResponseFine",
    "If response is larger then this, then initiate loop closure search at "
    "the fine resolution.",
    0.8, GetParameterManager());

  //////////////////////////////////////////////////////////////////////////////
  //    CorrelationParameters correlationParameters;

  m_pCorrelationSearchSpaceDimension = new Parameter<kt_double>(
    "CorrelationSearchSpaceDimension",
    "The size of the search grid used by the matcher. The search grid will "
    "have the size CorrelationSearchSpaceDimension * "
    "CorrelationSearchSpaceDimension",
    0.3, GetParameterManager());

  m_pCorrelationSearchSpaceResolution = new Parameter<kt_double>(
    "CorrelationSearchSpaceResolution",
    "The resolution (size of a grid cell) of the correlation grid.",
    0.01, GetParameterManager());

  m_pCorrelationSearchSpaceSmearDeviation = new Parameter<kt_double>(
    "CorrelationSearchSpaceSmearDeviation",
    "The point readings are smeared by this value in X and Y to create a "
    "smoother response.",
    0.03, GetParameterManager());


  //////////////////////////////////////////////////////////////////////////////
  //    CorrelationParameters loopCorrelationParameters;

  m_pLoopSearchSpaceDimension = new Parameter<kt_double>(
    "LoopSearchSpaceDimension",
    "The size of the search grid used by the matcher.",
    8.0, GetParameterManager());

  m_pLoopSearchSpaceResolution = new Parameter<kt_double>(
    "LoopSearchSpaceResolution",
    "The resolution (size of a grid cell) of the correlation grid.",
    0.05, GetParameterManager());

  m_pLoopSearchSpaceSmearDeviation = new Parameter<kt_double>(
    "LoopSearchSpaceSmearDeviation",
    "The point readings are smeared by this value in X and Y to create a "
    "smoother response.",
    0.03, GetParameterManager());

  //////////////////////////////////////////////////////////////////////////////
  // ScanMatcherParameters;

  m_pDistanceVariancePenalty = new Parameter<kt_double>(
    "DistanceVariancePenalty",
    "Variance of penalty for deviating from odometry when scan-matching. "
    "The penalty is a multiplier (less than 1.0) is a function of the "
    "delta of the scan position being tested and the odometric pose.",
    math::Square(0.3), GetParameterManager());

  m_pAngleVariancePenalty = new Parameter<kt_double>(
    "AngleVariancePenalty",
    "See DistanceVariancePenalty.",
    math::Square(math::DegreesToRadians(20)), GetParameterManager());

  m_pFineSearchAngleOffset = new Parameter<kt_double>(
    "FineSearchAngleOffset",
    "The range of angles to search during a fine search.",
    math::DegreesToRadians(0.2), GetParameterManager());

  m_pCoarseSearchAngleOffset = new Parameter<kt_double>(
    "CoarseSearchAngleOffset",
    "The range of angles to search during a coarse search.",
    math::DegreesToRadians(20), GetParameterManager());

  m_pCoarseAngleResolution = new Parameter<kt_double>(
    "CoarseAngleResolution",
    "Resolution of angles to search during a coarse search.",
    math::DegreesToRadians(2), GetParameterManager());

  m_pMinimumAnglePenalty = new Parameter<kt_double>(
    "MinimumAnglePenalty",
    "Minimum value of the angle penalty multiplier so scores do not become "
    "too small.",
    0.9, GetParameterManager());

  m_pMinimumDistancePenalty = new Parameter<kt_double>(
    "MinimumDistancePenalty",
    "Minimum value of the distance penalty multiplier so scores do not "
    "become too small.",
    0.5, GetParameterManager());
  
  m_pUseResponseExpansion = new Parameter<kt_bool>(
    "UseResponseExpansion",
    "Whether to increase the search space if no good matches are initially "
    "found.",
    false, GetParameterManager());

  m_pMinPassThrough = new Parameter<kt_int32u>(
    "MinPassThrough",
    "Number of beams that must pass through a cell before it will be considered to be occupied "
    "or unoccupied.  This prevents stray beams from messing up the map. "
    "found.",
    2, GetParameterManager());
  
  m_pOccupancyThreshold = new Parameter<kt_double>(
    "OccupancyThreshold",
    "Minimum ratio of beams hitting cell to beams passing through cell to be marked as occupied",
    0.1, GetParameterManager());

  m_pUseReflectors = new Parameter<kt_bool>(
    "UseReflectors",
    "Whether sequential scan matching also matches the scans' reflector points.",
    false, GetParameterManager());

  m_pUseGeometry = new Parameter<kt_bool>(
    "UseGeometry",
    "Whether scan matching uses the geometry; false matches on reflectors only.",
    true, GetParameterManager());

  m_pReflectorWeight = new Parameter<kt_double>(
    "ReflectorWeight",
    "Weight of the reflector response against the geometry response.",
    1.0, GetParameterManager());

  m_pReflectorSmearDeviation = new Parameter<kt_double>(
    "ReflectorSmearDeviation",
    "Smear deviation of the reflector correlation grid.",
    0.02, GetParameterManager());

  m_pReflectorResponseCap = new Parameter<kt_double>(
    "ReflectorResponseCap",
    "Summed weighted reflector hits at which the reflector response saturates at 1.",
    3.0, GetParameterManager());

  m_pReflectorInformationFusion = new Parameter<kt_bool>(
    "ReflectorInformationFusion",
    "Fuse separate geometry and reflector estimates by their covariances.",
    false, GetParameterManager());

  m_pReflectorCovarianceWindow = new Parameter<kt_double>(
    "ReflectorCovarianceWindow",
    "Relative response window for the response-surface covariances.",
    0.9, GetParameterManager());

  m_pReflectorConsistencyGateChi2 = new Parameter<kt_double>(
    "ReflectorConsistencyGateChi2",
    "chi^2 bound on the disagreement of the geometry and reflector estimates; 0 disables it.",
    11.34, GetParameterManager());

  m_pReflectorFuseHeading = new Parameter<kt_bool>(
    "ReflectorFuseHeading",
    "Whether the reflector estimate informs heading in information fusion.",
    true, GetParameterManager());

  m_pReflectorsInLoopClosure = new Parameter<kt_bool>(
    "ReflectorsInLoopClosure",
    "Whether loop closure uses the reflectors too.",
    false, GetParameterManager());

  m_pLoopMatchRelativeCovariance = new Parameter<kt_bool>(
    "LoopMatchRelativeCovariance",
    "Loop matcher coarse covariance from the relative-window 3x3 moments.",
    false, GetParameterManager());

  m_pLoopClosureCoarseCovariance = new Parameter<kt_bool>(
    "LoopClosureCoarseCovariance",
    "Closure edge covariance keeps the coarse search ambiguity (fine + coarse).",
    false, GetParameterManager());

  m_pLoopClosureMoveScan = new Parameter<kt_bool>(
    "LoopClosureMoveScan",
    "Whether an accepted closure moves the scan before the solve.",
    true, GetParameterManager());

  m_pLoopClosureGateChi2 = new Parameter<kt_double>(
    "LoopClosureGateChi2",
    "chi^2 gate of a closure against the current graph; 0 disables it.",
    0.0, GetParameterManager());

  m_pLoopClosureGateDriftXY = new Parameter<kt_double>(
    "LoopClosureGateDriftXY",
    "Assumed position drift per metre of path for the closure gate.",
    0.05, GetParameterManager());

  m_pOdometryPriorFusion = new Parameter<kt_bool>(
    "OdometryPriorFusion",
    "Fuse the sequential match with the odometry prediction by information (REFLECTOR_SLAM.md 5.10, S2).",
    false, GetParameterManager());

  m_pOdometryPriorStddevXyPerMeter = new Parameter<kt_double>(
    "OdometryPriorStddevXyPerMeter",
    "Odometry prior: position standard deviation per metre driven.",
    0.05, GetParameterManager());

  m_pOdometryPriorStddevXyMin = new Parameter<kt_double>(
    "OdometryPriorStddevXyMin",
    "Odometry prior: position standard deviation floor (m).",
    0.005, GetParameterManager());

  m_pOdometryPriorStddevYawPerRadian = new Parameter<kt_double>(
    "OdometryPriorStddevYawPerRadian",
    "Odometry prior: heading standard deviation per radian turned.",
    0.05, GetParameterManager());

  m_pOdometryPriorStddevYawPerMeter = new Parameter<kt_double>(
    "OdometryPriorStddevYawPerMeter",
    "Odometry prior: heading standard deviation per metre driven (rad/m).",
    0.01, GetParameterManager());

  m_pOdometryPriorStddevYawMin = new Parameter<kt_double>(
    "OdometryPriorStddevYawMin",
    "Odometry prior: heading standard deviation floor (rad).",
    0.002, GetParameterManager());

  m_pOdometryPriorGateChi2 = new Parameter<kt_double>(
    "OdometryPriorGateChi2",
    "Odometry prior: no fusion when the scan and odometry disagree beyond this chi^2 (3 DOF); 0 = always fuse.",
    11.34, GetParameterManager());

  m_pOdometryPriorStddevCrossPerMeter = new Parameter<kt_double>(
    "OdometryPriorStddevCrossPerMeter",
    "Odometry prior: position standard deviation across the direction of travel per metre.",
    0.05, GetParameterManager());

  m_pOdometryPriorFuseHeading = new Parameter<kt_bool>(
    "OdometryPriorFuseHeading",
    "Odometry prior also informs heading; off: position only, at the scan's heading.",
    false, GetParameterManager());

  m_pNearChainMinimumResponseCoarse = new Parameter<kt_double>(
    "NearChainMinimumResponseCoarse",
    "Coarse response a gated near-chain match needs (the loop closure's gate is separate).",
    0.35, GetParameterManager());

  m_pLinkChainScans = new Parameter<kt_int32u>(
    "LinkChainScans",
    "A chain match is linked to this many of the chain's closest scans (1: the closest only).",
    1, GetParameterManager());

  m_pNearChainGated = new Parameter<kt_bool>(
    "NearChainGated",
    "Match near chains like loop closures (wide coarse search and the loop-closure gates).",
    false, GetParameterManager());

  m_pLoopClosureGateDriftYaw = new Parameter<kt_double>(
    "LoopClosureGateDriftYaw",
    "Assumed heading drift (rad) per metre of path for the closure gate.",
    0.01, GetParameterManager());
}
/* Adding in getters and setters here for easy parameter access */

// General Parameters

bool Mapper::getParamUseScanMatching()
{
  return static_cast<bool>(m_pUseScanMatching->GetValue());
}

bool Mapper::getParamUseScanBarycenter()
{
  return static_cast<bool>(m_pUseScanBarycenter->GetValue());
}

double Mapper::getParamMinimumTimeInterval()
{
  return static_cast<double>(m_pMinimumTimeInterval->GetValue());
}

double Mapper::getParamMinimumTravelDistance()
{
  return static_cast<double>(m_pMinimumTravelDistance->GetValue());
}

double Mapper::getParamMinimumTravelHeading()
{
  return math::RadiansToDegrees(static_cast<double>(m_pMinimumTravelHeading->GetValue()));
}

int Mapper::getParamScanBufferSize()
{
  return static_cast<int>(m_pScanBufferSize->GetValue());
}

double Mapper::getParamScanBufferMaximumScanDistance()
{
  return static_cast<double>(m_pScanBufferMaximumScanDistance->GetValue());
}

double Mapper::getParamLinkMatchMinimumResponseFine()
{
  return static_cast<double>(m_pLinkMatchMinimumResponseFine->GetValue());
}

double Mapper::getParamLinkScanMaximumDistance()
{
  return static_cast<double>(m_pLinkScanMaximumDistance->GetValue());
}

double Mapper::getParamLoopSearchMaximumDistance()
{
  return static_cast<double>(m_pLoopSearchMaximumDistance->GetValue());
}

bool Mapper::getParamDoLoopClosing()
{
  return static_cast<bool>(m_pDoLoopClosing->GetValue());
}

int Mapper::getParamLoopMatchMinimumChainSize()
{
  return static_cast<int>(m_pLoopMatchMinimumChainSize->GetValue());
}

bool Mapper::getParamEnablePeriodicPoseGraphSolve()
{
  return static_cast<bool>(m_pEnablePeriodicPoseGraphSolve->GetValue());
}

int Mapper::getParamPeriodicSolveIntervalNodes()
{
  return static_cast<int>(m_pPeriodicSolveIntervalNodes->GetValue());
}

double Mapper::getParamLoopMatchMaximumVarianceCoarse()
{
  return static_cast<double>(std::sqrt(m_pLoopMatchMaximumVarianceCoarse->GetValue()));
}

double Mapper::getParamLoopMatchMinimumResponseCoarse()
{
  return static_cast<double>(m_pLoopMatchMinimumResponseCoarse->GetValue());
}

double Mapper::getParamLoopMatchMinimumResponseFine()
{
  return static_cast<double>(m_pLoopMatchMinimumResponseFine->GetValue());
}

// Correlation Parameters - Correlation Parameters

double Mapper::getParamCorrelationSearchSpaceDimension()
{
  return static_cast<double>(m_pCorrelationSearchSpaceDimension->GetValue());
}

double Mapper::getParamCorrelationSearchSpaceResolution()
{
  return static_cast<double>(m_pCorrelationSearchSpaceResolution->GetValue());
}

double Mapper::getParamCorrelationSearchSpaceSmearDeviation()
{
  return static_cast<double>(m_pCorrelationSearchSpaceSmearDeviation->GetValue());
}

// Correlation Parameters - Loop Correlation Parameters

double Mapper::getParamLoopSearchSpaceDimension()
{
  return static_cast<double>(m_pLoopSearchSpaceDimension->GetValue());
}

double Mapper::getParamLoopSearchSpaceResolution()
{
  return static_cast<double>(m_pLoopSearchSpaceResolution->GetValue());
}

double Mapper::getParamLoopSearchSpaceSmearDeviation()
{
  return static_cast<double>(m_pLoopSearchSpaceSmearDeviation->GetValue());
}

// ScanMatcher Parameters

double Mapper::getParamDistanceVariancePenalty()
{
  return std::sqrt(static_cast<double>(m_pDistanceVariancePenalty->GetValue()));
}

double Mapper::getParamAngleVariancePenalty()
{
  return std::sqrt(static_cast<double>(m_pAngleVariancePenalty->GetValue()));
}

double Mapper::getParamFineSearchAngleOffset()
{
  return static_cast<double>(m_pFineSearchAngleOffset->GetValue());
}

double Mapper::getParamCoarseSearchAngleOffset()
{
  return static_cast<double>(m_pCoarseSearchAngleOffset->GetValue());
}

double Mapper::getParamCoarseAngleResolution()
{
  return static_cast<double>(m_pCoarseAngleResolution->GetValue());
}

double Mapper::getParamMinimumAnglePenalty()
{
  return static_cast<double>(m_pMinimumAnglePenalty->GetValue());
}

double Mapper::getParamMinimumDistancePenalty()
{
  return static_cast<double>(m_pMinimumDistancePenalty->GetValue());
}

bool Mapper::getParamUseResponseExpansion()
{
  return static_cast<bool>(m_pUseResponseExpansion->GetValue());
}

int Mapper::getParamMinPassThrough()
{
  return static_cast<int>(m_pMinPassThrough->GetValue());
}

double Mapper::getParamOccupancyThreshold()
{
  return static_cast<double>(m_pOccupancyThreshold->GetValue());
}

bool Mapper::getParamUseReflectors()
{
  return static_cast<bool>(m_pUseReflectors->GetValue());
}

bool Mapper::getParamUseGeometry()
{
  return static_cast<bool>(m_pUseGeometry->GetValue());
}

double Mapper::getParamReflectorWeight()
{
  return static_cast<double>(m_pReflectorWeight->GetValue());
}

double Mapper::getParamReflectorSmearDeviation()
{
  return static_cast<double>(m_pReflectorSmearDeviation->GetValue());
}

double Mapper::getParamReflectorResponseCap()
{
  return static_cast<double>(m_pReflectorResponseCap->GetValue());
}

bool Mapper::getParamReflectorInformationFusion()
{
  return static_cast<bool>(m_pReflectorInformationFusion->GetValue());
}

double Mapper::getParamReflectorCovarianceWindow()
{
  return static_cast<double>(m_pReflectorCovarianceWindow->GetValue());
}

double Mapper::getParamReflectorConsistencyGateChi2()
{
  return static_cast<double>(m_pReflectorConsistencyGateChi2->GetValue());
}

bool Mapper::getParamReflectorFuseHeading()
{
  return static_cast<bool>(m_pReflectorFuseHeading->GetValue());
}

bool Mapper::getParamReflectorsInLoopClosure()
{
  return static_cast<bool>(m_pReflectorsInLoopClosure->GetValue());
}
bool Mapper::getParamLoopMatchRelativeCovariance()
{
  return static_cast<bool>(m_pLoopMatchRelativeCovariance->GetValue());
}

bool Mapper::getParamLoopClosureCoarseCovariance()
{
  return static_cast<bool>(m_pLoopClosureCoarseCovariance->GetValue());
}

bool Mapper::getParamNearChainGated()
{
  return static_cast<bool>(m_pNearChainGated->GetValue());
}

bool Mapper::getParamLoopClosureMoveScan()
{
  return static_cast<bool>(m_pLoopClosureMoveScan->GetValue());
}

double Mapper::getParamLoopClosureGateChi2()
{
  return static_cast<double>(m_pLoopClosureGateChi2->GetValue());
}

double Mapper::getParamLoopClosureGateDriftXY()
{
  return static_cast<double>(m_pLoopClosureGateDriftXY->GetValue());
}

double Mapper::getParamLoopClosureGateDriftYaw()
{
  return static_cast<double>(m_pLoopClosureGateDriftYaw->GetValue());
}

/* Setters for parameters */
// General Parameters
void Mapper::setParamUseScanMatching(bool b)
{
  m_pUseScanMatching->SetValue((kt_bool)b);
}

void Mapper::setParamUseScanBarycenter(bool b)
{
  m_pUseScanBarycenter->SetValue((kt_bool)b);
}

void Mapper::setParamMinimumTimeInterval(double d)
{
  m_pMinimumTimeInterval->SetValue((kt_double)d);
}

void Mapper::setParamMinimumTravelDistance(double d)
{
  m_pMinimumTravelDistance->SetValue((kt_double)d);
}

void Mapper::setParamMinimumTravelHeading(double d)
{
  m_pMinimumTravelHeading->SetValue((kt_double)d);
}

void Mapper::setParamScanBufferSize(int i)
{
  m_pScanBufferSize->SetValue((kt_int32u)i);
}

void Mapper::setParamScanBufferMaximumScanDistance(double d)
{
  m_pScanBufferMaximumScanDistance->SetValue((kt_double)d);
}

void Mapper::setParamLinkMatchMinimumResponseFine(double d)
{
  m_pLinkMatchMinimumResponseFine->SetValue((kt_double)d);
}

void Mapper::setParamLinkScanMaximumDistance(double d)
{
  m_pLinkScanMaximumDistance->SetValue((kt_double)d);
}

void Mapper::setParamLoopSearchMaximumDistance(double d)
{
  m_pLoopSearchMaximumDistance->SetValue((kt_double)d);
}

void Mapper::setParamDoLoopClosing(bool b)
{
  m_pDoLoopClosing->SetValue((kt_bool)b);
}

void Mapper::setParamLoopMatchMinimumChainSize(int i)
{
  m_pLoopMatchMinimumChainSize->SetValue((kt_int32u)i);
}

void Mapper::setParamEnablePeriodicPoseGraphSolve(bool b)
{
  m_pEnablePeriodicPoseGraphSolve->SetValue((kt_bool)b);
}

void Mapper::setParamPeriodicSolveIntervalNodes(int i)
{
  m_pPeriodicSolveIntervalNodes->SetValue((kt_int32u)i);
}

void Mapper::setParamLoopMatchMaximumVarianceCoarse(double d)
{
  m_pLoopMatchMaximumVarianceCoarse->SetValue((kt_double)math::Square(d));
}

void Mapper::setParamLoopMatchMinimumResponseCoarse(double d)
{
  m_pLoopMatchMinimumResponseCoarse->SetValue((kt_double)d);
}

void Mapper::setParamLoopMatchMinimumResponseFine(double d)
{
  m_pLoopMatchMinimumResponseFine->SetValue((kt_double)d);
}

// Correlation Parameters - Correlation Parameters
void Mapper::setParamCorrelationSearchSpaceDimension(double d)
{
  m_pCorrelationSearchSpaceDimension->SetValue((kt_double)d);
}

void Mapper::setParamCorrelationSearchSpaceResolution(double d)
{
  m_pCorrelationSearchSpaceResolution->SetValue((kt_double)d);
}

void Mapper::setParamCorrelationSearchSpaceSmearDeviation(double d)
{
  m_pCorrelationSearchSpaceSmearDeviation->SetValue((kt_double)d);
}


// Correlation Parameters - Loop Closure Parameters
void Mapper::setParamLoopSearchSpaceDimension(double d)
{
  m_pLoopSearchSpaceDimension->SetValue((kt_double)d);
}

void Mapper::setParamLoopSearchSpaceResolution(double d)
{
  m_pLoopSearchSpaceResolution->SetValue((kt_double)d);
}

void Mapper::setParamLoopSearchSpaceSmearDeviation(double d)
{
  m_pLoopSearchSpaceSmearDeviation->SetValue((kt_double)d);
}


// Scan Matcher Parameters
void Mapper::setParamDistanceVariancePenalty(double d)
{
  m_pDistanceVariancePenalty->SetValue((kt_double)math::Square(d));
}

void Mapper::setParamAngleVariancePenalty(double d)
{
  m_pAngleVariancePenalty->SetValue((kt_double)math::Square(d));
}

void Mapper::setParamFineSearchAngleOffset(double d)
{
  m_pFineSearchAngleOffset->SetValue((kt_double)d);
}

void Mapper::setParamCoarseSearchAngleOffset(double d)
{
  m_pCoarseSearchAngleOffset->SetValue((kt_double)d);
}

void Mapper::setParamCoarseAngleResolution(double d)
{
  m_pCoarseAngleResolution->SetValue((kt_double)d);
}

void Mapper::setParamMinimumAnglePenalty(double d)
{
  m_pMinimumAnglePenalty->SetValue((kt_double)d);
}

void Mapper::setParamMinimumDistancePenalty(double d)
{
  m_pMinimumDistancePenalty->SetValue((kt_double)d);
}

void Mapper::setParamUseResponseExpansion(bool b)
{
  m_pUseResponseExpansion->SetValue((kt_bool)b);
}

void Mapper::setParamMinPassThrough(int i)
{
  m_pMinPassThrough->SetValue((kt_int32u)i);
}

void Mapper::setParamOccupancyThreshold(double d)
{
  m_pOccupancyThreshold->SetValue((kt_double)d);
}

void Mapper::setParamUseReflectors(bool b)
{
  m_pUseReflectors->SetValue((kt_bool)b);
}

void Mapper::setParamUseGeometry(bool b)
{
  m_pUseGeometry->SetValue((kt_bool)b);
}

void Mapper::setParamReflectorWeight(double d)
{
  m_pReflectorWeight->SetValue((kt_double)d);
}

void Mapper::setParamReflectorSmearDeviation(double d)
{
  m_pReflectorSmearDeviation->SetValue((kt_double)d);
}

void Mapper::setParamReflectorResponseCap(double d)
{
  m_pReflectorResponseCap->SetValue((kt_double)d);
}

void Mapper::setParamReflectorInformationFusion(bool b)
{
  m_pReflectorInformationFusion->SetValue((kt_bool)b);
}

void Mapper::setParamReflectorCovarianceWindow(double d)
{
  m_pReflectorCovarianceWindow->SetValue((kt_double)d);
}

void Mapper::setParamReflectorConsistencyGateChi2(double d)
{
  m_pReflectorConsistencyGateChi2->SetValue((kt_double)d);
}

void Mapper::setParamReflectorFuseHeading(bool b)
{
  m_pReflectorFuseHeading->SetValue((kt_bool)b);
}

void Mapper::setParamReflectorsInLoopClosure(bool b)
{
  m_pReflectorsInLoopClosure->SetValue((kt_bool)b);
}
void Mapper::setParamLoopMatchRelativeCovariance(bool b)
{
  m_pLoopMatchRelativeCovariance->SetValue((kt_bool)b);
}

void Mapper::setParamLoopClosureCoarseCovariance(bool b)
{
  m_pLoopClosureCoarseCovariance->SetValue((kt_bool)b);
}

void Mapper::setParamOdometryPriorFusion(bool v)
{
  m_pOdometryPriorFusion->SetValue((kt_bool)v);
}

void Mapper::setParamOdometryPriorStddevXyPerMeter(double v)
{
  m_pOdometryPriorStddevXyPerMeter->SetValue((kt_double)v);
}

void Mapper::setParamOdometryPriorStddevXyMin(double v)
{
  m_pOdometryPriorStddevXyMin->SetValue((kt_double)v);
}

void Mapper::setParamOdometryPriorStddevYawPerRadian(double v)
{
  m_pOdometryPriorStddevYawPerRadian->SetValue((kt_double)v);
}

void Mapper::setParamOdometryPriorStddevYawPerMeter(double v)
{
  m_pOdometryPriorStddevYawPerMeter->SetValue((kt_double)v);
}

void Mapper::setParamOdometryPriorStddevYawMin(double v)
{
  m_pOdometryPriorStddevYawMin->SetValue((kt_double)v);
}

void Mapper::setParamOdometryPriorGateChi2(double v)
{
  m_pOdometryPriorGateChi2->SetValue((kt_double)v);
}

void Mapper::setParamOdometryPriorStddevCrossPerMeter(double v)
{
  m_pOdometryPriorStddevCrossPerMeter->SetValue(v);
}

void Mapper::setParamOdometryPriorFuseHeading(bool v)
{
  m_pOdometryPriorFuseHeading->SetValue((kt_bool)v);
}

void Mapper::setParamNearChainMinimumResponseCoarse(double v)
{
  m_pNearChainMinimumResponseCoarse->SetValue(v);
}

void Mapper::setParamLinkChainScans(int n)
{
  m_pLinkChainScans->SetValue(static_cast<kt_int32u>(std::max(n, 1)));
}

void Mapper::setParamNearChainGated(bool b)
{
  m_pNearChainGated->SetValue((kt_bool)b);
}

void Mapper::setParamLoopClosureMoveScan(bool b)
{
  m_pLoopClosureMoveScan->SetValue((kt_bool)b);
}

void Mapper::setParamLoopClosureGateChi2(double d)
{
  m_pLoopClosureGateChi2->SetValue((kt_double)d);
}

void Mapper::setParamLoopClosureGateDriftXY(double d)
{
  m_pLoopClosureGateDriftXY->SetValue((kt_double)d);
}

void Mapper::setParamLoopClosureGateDriftYaw(double d)
{
  m_pLoopClosureGateDriftYaw->SetValue((kt_double)d);
}


void Mapper::Initialize(kt_double rangeThreshold)
{
  if (m_Initialized) {
    return;
  }
  // create sequential scan and loop matcher, update if deserialized

  if (m_pSequentialScanMatcher) {
    delete m_pSequentialScanMatcher;
  }
  m_pSequentialScanMatcher = ScanMatcher::Create(this,
    m_pCorrelationSearchSpaceDimension->GetValue(),
    m_pCorrelationSearchSpaceResolution->GetValue(),
    m_pCorrelationSearchSpaceSmearDeviation->GetValue(),
    rangeThreshold);
  assert(m_pSequentialScanMatcher);

  if (m_Deserialized) {
    m_pMapperSensorManager->SetRunningScanBufferSize(m_pScanBufferSize->GetValue());
    m_pMapperSensorManager->SetRunningScanBufferMaximumDistance(m_pScanBufferMaximumScanDistance->GetValue());

    m_pGraph->UpdateLoopScanMatcher(rangeThreshold);
  } else {
    m_pMapperSensorManager = new MapperSensorManager(m_pScanBufferSize->GetValue(),
      m_pScanBufferMaximumScanDistance->GetValue());

    m_pGraph = new MapperGraph(this, rangeThreshold);
  }

  m_Initialized = true;
}

void Mapper::SaveToFile(const std::string & filename)
{
  printf("Save To File %s \n", filename.c_str());
  std::ofstream ofs(filename.c_str());
  boost::archive::binary_oarchive oa(ofs, boost::archive::no_codecvt);
  oa << BOOST_SERIALIZATION_NVP(*this);
}

void Mapper::LoadFromFile(const std::string & filename)
{
  printf("Load From File %s \n", filename.c_str());
  std::ifstream ifs(filename.c_str());
  boost::archive::binary_iarchive ia(ifs, boost::archive::no_codecvt);
  ia >> BOOST_SERIALIZATION_NVP(*this);
  m_Deserialized = true;
  m_Initialized = false;
}

void Mapper::Reset()
{
  if (m_pSequentialScanMatcher) {
    delete m_pSequentialScanMatcher;
    m_pSequentialScanMatcher = NULL;
  }
  if (m_pGraph) {
    delete m_pGraph;
    m_pGraph = NULL;
  }
  if (m_pMapperSensorManager) {
    delete m_pMapperSensorManager;
    m_pMapperSensorManager = NULL;
  }
  m_Initialized = false;
  m_Deserialized = false;
  while (!m_LocalizationScanVertices.empty()) {
    m_LocalizationScanVertices.pop();
  }
}

kt_bool Mapper::Process(Object *  /*pObject*/)  // NOLINT
{
  return true;
}

kt_bool Mapper::Process(LocalizedRangeScan * pScan, Matrix3 * covariance)
{
  if (pScan != NULL) {
    karto::LaserRangeFinder * pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL || pLaserRangeFinder->Validate(pScan) == false) {
      return false;
    }

    if (m_Initialized == false) {
      // initialize mapper with range threshold from device
      Initialize(pLaserRangeFinder->GetRangeThreshold());
    }

    // get last scan
    LocalizedRangeScan * pLastScan = m_pMapperSensorManager->GetLastScan(pScan->GetSensorName());

    // update scans corrected pose based on last correction
    if (pLastScan != NULL) {
      Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
      pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));
    }

    // test if scan is outside minimum boundary or if heading is larger then minimum heading
    if (!HasMovedEnough(pScan, pLastScan)) {
      return false;
    }

    Matrix3 cov;
    cov.SetToIdentity();

    // correct scan (if not first scan)
    if (m_pUseScanMatching->GetValue() && pLastScan != NULL) {
      Pose2 bestPose;
      m_pSequentialScanMatcher->MatchScan(pScan,
        m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
        bestPose,
        cov);
      pScan->SetSensorPose(bestPose);
      if (covariance) {
        *covariance = cov;
      }
    }

    // add scan to buffer and assign id
    m_pMapperSensorManager->AddScan(pScan);

    if (m_pUseScanMatching->GetValue()) {
      // add to graph
      m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, cov);

      m_pMapperSensorManager->AddRunningScan(pScan);

      if (m_pDoLoopClosing->GetValue()) {
        std::vector<Name> deviceNames = m_pMapperSensorManager->GetSensorNames();
        const_forEach(std::vector<Name>, &deviceNames)
        {
          m_pGraph->TryCloseLoop(pScan, *iter);
        }
      }
    }

    m_pMapperSensorManager->SetLastScan(pScan);

    return true;
  }

  return false;
}

kt_bool Mapper::ProcessAgainstNodesNearBy(LocalizedRangeScan * pScan, kt_bool addScanToLocalizationBuffer, Matrix3 * covariance)
{
  if (pScan != NULL) {
    karto::LaserRangeFinder * pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL ||
      pLaserRangeFinder->Validate(pScan) == false)
    {
      return false;
    }

    if (m_Initialized == false) {
      // initialize mapper with range threshold from device
      Initialize(pLaserRangeFinder->GetRangeThreshold());
    }

    Vertex<LocalizedRangeScan> * closetVertex = m_pGraph->FindNearByScan(
      pScan->GetSensorName(), pScan->GetOdometricPose());
    LocalizedRangeScan * pLastScan = NULL;
    if (closetVertex) {
      pLastScan = m_pMapperSensorManager->GetScan(pScan->GetSensorName(),
          closetVertex->GetObject()->GetStateId());
      m_pMapperSensorManager->ClearRunningScans(pScan->GetSensorName());
      m_pMapperSensorManager->AddRunningScan(pLastScan);
      m_pMapperSensorManager->SetLastScan(pLastScan);
    }

    Matrix3 cov;
    cov.SetToIdentity();

    // correct scan (if not first scan)
    if (m_pUseScanMatching->GetValue() && pLastScan != NULL) {
      Pose2 bestPose;
      m_pSequentialScanMatcher->MatchScan(pScan,
        m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
        bestPose,
        cov);
      pScan->SetSensorPose(bestPose);
    }

    pScan->SetOdometricPose(pScan->GetCorrectedPose());

    if (covariance) {
      *covariance = cov;
    }

    // add scan to buffer and assign id
    m_pMapperSensorManager->AddScan(pScan);

    Vertex<LocalizedRangeScan> * scan_vertex = NULL;
    if (m_pUseScanMatching->GetValue()) {
      // add to graph
      scan_vertex = m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, cov);

      m_pMapperSensorManager->AddRunningScan(pScan);

      if (m_pDoLoopClosing->GetValue()) {
        std::vector<Name> deviceNames =
          m_pMapperSensorManager->GetSensorNames();
        const_forEach(std::vector<Name>, &deviceNames)
        {
          m_pGraph->TryCloseLoop(pScan, *iter);
        }
      }
    }

    m_pMapperSensorManager->SetLastScan(pScan);

    if (addScanToLocalizationBuffer) {
      AddScanToLocalizationBuffer(pScan, scan_vertex);
    }

    return true;
  }

  return false;
}

kt_bool Mapper::ProcessLocalization(LocalizedRangeScan * pScan, Matrix3 * covariance)
{
  if (pScan == NULL) {
    return false;
  }

  karto::LaserRangeFinder * pLaserRangeFinder = pScan->GetLaserRangeFinder();

  // validate scan
  if (pLaserRangeFinder == NULL || pScan == NULL ||
    pLaserRangeFinder->Validate(pScan) == false)
  {
    return false;
  }

  if (m_Initialized == false) {
    // initialize mapper with range threshold from device
    Initialize(pLaserRangeFinder->GetRangeThreshold());
  }

  // get last scan
  LocalizedRangeScan * pLastScan = m_pMapperSensorManager->GetLastScan(
    pScan->GetSensorName());

  // update scans corrected pose based on last correction
  if (pLastScan != NULL) {
    Transform lastTransform(pLastScan->GetOdometricPose(),
      pLastScan->GetCorrectedPose());
    pScan->SetCorrectedPose(lastTransform.TransformPose(
        pScan->GetOdometricPose()));
  }

  // test if scan is outside minimum boundary
  // or if heading is larger then minimum heading
  if (!HasMovedEnough(pScan, pLastScan)) {
    return false;
  }

  Matrix3 cov;
  cov.SetToIdentity();

  // correct scan (if not first scan)
  if (m_pUseScanMatching->GetValue() && pLastScan != NULL) {
    Pose2 bestPose;
    m_pSequentialScanMatcher->MatchScan(pScan,
      m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
      bestPose,
      cov);
    pScan->SetSensorPose(bestPose);
    if (covariance) {
      *covariance = cov;
    }
  }

  // add scan to buffer and assign id
  m_pMapperSensorManager->AddScan(pScan);

  Vertex<LocalizedRangeScan> * scan_vertex = NULL;
  if (m_pUseScanMatching->GetValue()) {
    // add to graph
    scan_vertex = m_pGraph->AddVertex(pScan);
    m_pGraph->AddEdges(pScan, cov);

    m_pMapperSensorManager->AddRunningScan(pScan);

    if (m_pDoLoopClosing->GetValue()) {
      std::vector<Name> deviceNames = m_pMapperSensorManager->GetSensorNames();
      const_forEach(std::vector<Name>, &deviceNames)
      {
        m_pGraph->TryCloseLoop(pScan, *iter);
      }
    }
  }

  m_pMapperSensorManager->SetLastScan(pScan);
  AddScanToLocalizationBuffer(pScan, scan_vertex);

  return true;
}

void Mapper::AddScanToLocalizationBuffer(LocalizedRangeScan * pScan, Vertex <LocalizedRangeScan> * scan_vertex)
{
  // generate the info to store and later decay, outside of dataset
  LocalizationScanVertex lsv;
  lsv.scan = pScan;
  lsv.vertex = scan_vertex;
  m_LocalizationScanVertices.push(lsv);

  if (m_LocalizationScanVertices.size() > getParamScanBufferSize()) {
    LocalizationScanVertex & oldLSV = m_LocalizationScanVertices.front();
    RemoveNodeFromGraph(oldLSV.vertex);

    // delete node and scans
    // free hat!
    // No need to delete from m_scans as those pointers will be freed memory
    oldLSV.vertex->RemoveObject();
    m_pMapperSensorManager->RemoveScan(oldLSV.scan);
    if (oldLSV.scan) {
      delete oldLSV.scan;
      oldLSV.scan = NULL;
    }

    m_LocalizationScanVertices.pop();
  }
}

void Mapper::ClearLocalizationBuffer()
{
  while (!m_LocalizationScanVertices.empty())
  {
    LocalizationScanVertex& oldLSV = m_LocalizationScanVertices.front();
    RemoveNodeFromGraph(oldLSV.vertex);
    oldLSV.vertex->RemoveObject();
    m_pMapperSensorManager->RemoveScan(oldLSV.scan);
    if (oldLSV.scan)
    {
      delete oldLSV.scan;
      oldLSV.scan = NULL;
    }

    m_LocalizationScanVertices.pop();
  }

  std::vector<Name> names = m_pMapperSensorManager->GetSensorNames();
  for (uint i = 0; i != names.size(); i++)
  {
    m_pMapperSensorManager->ClearRunningScans(names[i]);
    m_pMapperSensorManager->ClearLastScan(names[i]);
  }

  return;
}

kt_bool Mapper::RemoveNodeFromGraph(Vertex<LocalizedRangeScan> * vertex_to_remove)
{
  // 1) delete edges in adjacent vertices, graph, and optimizer
  std::vector<Vertex<LocalizedRangeScan> *> adjVerts =
    vertex_to_remove->GetAdjacentVertices();
  for (int i = 0; i != adjVerts.size(); i++) {
    std::vector<Edge<LocalizedRangeScan> *> adjEdges = adjVerts[i]->GetEdges();
    bool found = false;
    for (int j = 0; j != adjEdges.size(); j++) {
      if (adjEdges[j]->GetTarget() == vertex_to_remove ||
        adjEdges[j]->GetSource() == vertex_to_remove)
      {
        adjVerts[i]->RemoveEdge(j);
        m_pScanOptimizer->RemoveConstraint(
          adjEdges[j]->GetSource()->GetObject()->GetUniqueId(),
          adjEdges[j]->GetTarget()->GetObject()->GetUniqueId());
        std::vector<Edge<LocalizedRangeScan> *> edges = m_pGraph->GetEdges();
        std::vector<Edge<LocalizedRangeScan> *>::iterator edgeGraphIt =
          std::find(edges.begin(), edges.end(), adjEdges[j]);

        if (edgeGraphIt == edges.end()) {
          std::cout << "Edge not found in graph to remove!" << std::endl;
          continue;
        }

        int posEdge = edgeGraphIt - edges.begin();
        m_pGraph->RemoveEdge(posEdge);   // remove from graph
        delete *edgeGraphIt;   // free hat!
        *edgeGraphIt = NULL;
        found = true;
      }
    }
    if (!found) {
      std::cout << "Failed to find any edge in adj. vertex" <<
        " with a matching vertex to current!" << std::endl;
    }
  }

  // 2) delete vertex from optimizer
  m_pScanOptimizer->RemoveNode(vertex_to_remove->GetObject()->GetUniqueId());

  // 3) delete from vertex map
  std::map<Name, std::map<int, Vertex<LocalizedRangeScan> *>>
  vertexMap = m_pGraph->GetVertices();
  std::map<int, Vertex<LocalizedRangeScan> *> graphVertices =
    vertexMap[vertex_to_remove->GetObject()->GetSensorName()];
  std::map<int, Vertex<LocalizedRangeScan> *>::iterator
    vertexGraphIt = graphVertices.find(vertex_to_remove->GetObject()->GetStateId());
  if (vertexGraphIt != graphVertices.end()) {
    m_pGraph->RemoveVertex(vertex_to_remove->GetObject()->GetSensorName(),
      vertexGraphIt->second->GetObject()->GetStateId());
  } else {
    std::cout << "Vertex not found in graph to remove!" << std::endl;
    return false;
  }

  return true;
}

kt_bool Mapper::ProcessAgainstNode(
  LocalizedRangeScan * pScan,
  const int & nodeId,
  Matrix3 * covariance)
{
  if (pScan != NULL) {
    karto::LaserRangeFinder * pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL ||
      pLaserRangeFinder->Validate(pScan) == false)
    {
      return false;
    }

    if (m_Initialized == false) {
      // initialize mapper with range threshold from device
      Initialize(pLaserRangeFinder->GetRangeThreshold());
    }

    // If we're matching against a node from an older mapping session
    // lets get the first scan as the last scan and populate running scans
    // with the first few from that run as well.
    LocalizedRangeScan * pLastScan =
      m_pMapperSensorManager->GetScan(pScan->GetSensorName(), nodeId);
    m_pMapperSensorManager->ClearRunningScans(pScan->GetSensorName());
    m_pMapperSensorManager->AddRunningScan(pLastScan);
    m_pMapperSensorManager->SetLastScan(pLastScan);

    Matrix3 cov;
    cov.SetToIdentity();

    // correct scan (if not first scan)
    if (m_pUseScanMatching->GetValue() && pLastScan != NULL) {
      Pose2 bestPose;
      m_pSequentialScanMatcher->MatchScan(pScan,
        m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
        bestPose,
        cov);
      pScan->SetSensorPose(bestPose);
    }

    pScan->SetOdometricPose(pScan->GetCorrectedPose());
    if (covariance) {
      *covariance = cov;
    }

    // add scan to buffer and assign id
    m_pMapperSensorManager->AddScan(pScan);

    if (m_pUseScanMatching->GetValue()) {
      // add to graph
      m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, cov);

      m_pMapperSensorManager->AddRunningScan(pScan);

      if (m_pDoLoopClosing->GetValue()) {
        std::vector<Name> deviceNames =
          m_pMapperSensorManager->GetSensorNames();
        const_forEach(std::vector<Name>, &deviceNames)
        {
          m_pGraph->TryCloseLoop(pScan, *iter);
        }
      }
    }

    m_pMapperSensorManager->SetLastScan(pScan);

    return true;
  }

  return false;
}

kt_bool Mapper::ProcessAtDock(LocalizedRangeScan * pScan, Matrix3 * covariance)
{
  // Special case of processing against node where node is the starting point
  return ProcessAgainstNode(pScan, 0, covariance);
}

/**
 * Is the scan sufficiently far from the last scan?
 * @param pScan
 * @param pLastScan
 * @return true if the scans are sufficiently far
 */
kt_bool Mapper::HasMovedEnough(LocalizedRangeScan * pScan, LocalizedRangeScan * pLastScan) const
{
  // test if first scan
  if (pLastScan == NULL) {
    return true;
  }

  kt_double timeInterval = pScan->GetTime() - pLastScan->GetTime();
  Pose2 lastScannerPose = pLastScan->GetSensorAt(pLastScan->GetOdometricPose());
  Pose2 scannerPose = pScan->GetSensorAt(pScan->GetOdometricPose());

  return HasMovedEnough(scannerPose, lastScannerPose, timeInterval);
}

kt_bool Mapper::HasMovedEnough(const Pose2 & pose, const Pose2 & lastPose, kt_double timeInterval) const
{
  // test if enough time has passed
  if (timeInterval >= m_pMinimumTimeInterval->GetValue()) {
    return true;
  }

  // test if we have turned enough
  kt_double deltaHeading = math::NormalizeAngle(
    pose.GetHeading() - lastPose.GetHeading());
  if (fabs(deltaHeading) >= m_pMinimumTravelHeading->GetValue()) {
    return true;
  }

  // test if we have moved enough
  kt_double squaredTravelDistance = lastPose.GetPosition().SquaredDistance(
    pose.GetPosition());
  if (squaredTravelDistance >= math::Square(m_pMinimumTravelDistance->GetValue()) - KT_TOLERANCE) {
    return true;
  }

  return false;
}

/**
 * Gets all the processed scans
 * @return all scans
 */
const LocalizedRangeScanVector Mapper::GetAllProcessedScans() const
{
  LocalizedRangeScanVector allScans;

  if (m_pMapperSensorManager != NULL) {
    allScans = m_pMapperSensorManager->GetAllScans();
  }

  return allScans;
}

/**
 * Adds a listener
 * @param pListener
 */
void Mapper::AddListener(MapperListener * pListener)
{
  m_Listeners.push_back(pListener);
}

/**
 * Removes a listener
 * @param pListener
 */
void Mapper::RemoveListener(MapperListener * pListener)
{
  std::vector<MapperListener *>::iterator iter = std::find(m_Listeners.begin(),
      m_Listeners.end(), pListener);
  if (iter != m_Listeners.end()) {
    m_Listeners.erase(iter);
  }
}

void Mapper::FireInfo(const std::string & rInfo) const
{
  const_forEach(std::vector<MapperListener *>, &m_Listeners)
  {
    (*iter)->Info(rInfo);
  }
}

void Mapper::FireDebug(const std::string & rInfo) const
{
  const_forEach(std::vector<MapperListener *>, &m_Listeners)
  {
    MapperDebugListener * pListener = dynamic_cast<MapperDebugListener *>(*iter);

    if (pListener != NULL) {
      pListener->Debug(rInfo);
    }
  }
}

void Mapper::FireLoopClosureCheck(const std::string & rInfo) const
{
  const_forEach(std::vector<MapperListener *>, &m_Listeners)
  {
    MapperLoopClosureListener * pListener = dynamic_cast<MapperLoopClosureListener *>(*iter);

    if (pListener != NULL) {
      pListener->LoopClosureCheck(rInfo);
    }
  }
}

void Mapper::FireBeginLoopClosure(const std::string & rInfo) const
{
  const_forEach(std::vector<MapperListener *>, &m_Listeners)
  {
    MapperLoopClosureListener * pListener = dynamic_cast<MapperLoopClosureListener *>(*iter);

    if (pListener != NULL) {
      pListener->BeginLoopClosure(rInfo);
    }
  }
}

void Mapper::FireEndLoopClosure(const std::string & rInfo) const
{
  const_forEach(std::vector<MapperListener *>, &m_Listeners)
  {
    MapperLoopClosureListener * pListener = dynamic_cast<MapperLoopClosureListener *>(*iter);

    if (pListener != NULL) {
      pListener->EndLoopClosure(rInfo);
    }
  }
}

void Mapper::SetScanSolver(ScanSolver * pScanOptimizer)
{
  m_pScanOptimizer = pScanOptimizer;
}

ScanSolver * Mapper::getScanSolver()
{
  return m_pScanOptimizer;
}

MapperGraph * Mapper::GetGraph() const
{
  return m_pGraph;
}

ScanMatcher * Mapper::GetSequentialScanMatcher() const
{
  return m_pSequentialScanMatcher;
}

ScanMatcher * Mapper::GetLoopScanMatcher() const
{
  return m_pGraph->GetLoopScanMatcher();
}
}  // namespace karto
BOOST_CLASS_EXPORT(karto::BreadthFirstTraversal<karto::LocalizedRangeScan>)
