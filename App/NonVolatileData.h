#ifndef APP_NON_VOLATILE_DATA_H_
#define APP_NON_VOLATILE_DATA_H_

/* C++ */
#include <array>
#include <cstddef>
#include <cstdint>

/* Project */
#include "Config.h"
#include "Fram.h"

namespace NonVolatileData {
/* 不揮発メモリのアドレス算出構造体 */
#pragma pack(push, 1)
struct NonVolatileDataAddress {
  /* 1. ラインセンサー・マーカーセンサーのキャリブレーション情報 */
  struct SensorCalibrationData {
    /* ラインセンサー */
    std::array<uint16_t, 16> lineMin, lineMax;
    std::array<float, 16> lineCoeff;
    /* マーカーセンサー */
    std::array<uint16_t, 2> markerMax;
  } sensorCalibration;
  /* 2. 曲率記憶 */
  struct VelocityMappingData {
    uint16_t numPoints;
    std::array<uint16_t, kMappingMaxPoints> deltaDistance; /* [mm] */
    std::array<uint16_t, kMappingMaxPoints> deltaAngle;    /* [mrad] */
  } velocityMapping;
  /* 3. 補正位置 */
  struct PositionCorrectionData {
    uint16_t numCrossLinePoints;
    std::array<uint16_t, kCorrectionMaxPoints> crossLine; /* [mm] */
    uint16_t numCurveMarkerPoints;
    std::array<uint16_t, kCorrectionMaxPoints> curveMarker; /* [mm] */
  } positionCorrection;
  /* 4. ログ領域 */
  struct LogData {
    uint32_t bytes;
    uint8_t dummyLogData;
  } logData;
};
#pragma pack(pop)

static_assert(sizeof(NonVolatileDataAddress) <= Fram::kMaxAddress, "NonVolatileData <= FRAM Capacity");

/* 1. ラインセンサー・マーカーセンサーのキャリブレーション情報 */
static constexpr uint32_t kAddressSensorCalibrationDataLineMin =
    offsetof(NonVolatileDataAddress, sensorCalibration.lineMin);
static constexpr uint32_t kAddressSensorCalibrationDataLineMax =
    offsetof(NonVolatileDataAddress, sensorCalibration.lineMax);
static constexpr uint32_t kAddressSensorCalibrationDataLineCoeff =
    offsetof(NonVolatileDataAddress, sensorCalibration.lineCoeff);
static constexpr uint32_t kAddressSensorCalibrationDataMarkerMax =
    offsetof(NonVolatileDataAddress, sensorCalibration.markerMax);
/* 2. 曲率記憶 */
static constexpr uint32_t kAddressVelocityMappingDataNumPoints =
    offsetof(NonVolatileDataAddress, velocityMapping.numPoints);
static constexpr uint32_t kAddressVelocityMappingDataDeltaDistance =
    offsetof(NonVolatileDataAddress, velocityMapping.deltaDistance);
static constexpr uint32_t kAddressVelocityMappingDataDeltaAngle =
    offsetof(NonVolatileDataAddress, velocityMapping.deltaAngle);
/* 3. 補正位置 */
static constexpr uint32_t kAddressPositionCorrectionDataNumCrossLinePoints =
    offsetof(NonVolatileDataAddress, positionCorrection.numCrossLinePoints);
static constexpr uint32_t kAddressPositionCorrectionDataCrossLine =
    offsetof(NonVolatileDataAddress, positionCorrection.crossLine);
static constexpr uint32_t kAddressPositionCorrectionDataNumCurveMarkerPoints =
    offsetof(NonVolatileDataAddress, positionCorrection.numCurveMarkerPoints);
static constexpr uint32_t kAddressPositionCorrectionDataCurveMarker =
    offsetof(NonVolatileDataAddress, positionCorrection.curveMarker);
/* 4. ログ領域 */
static constexpr uint32_t kAddressLogDataBytes = offsetof(NonVolatileDataAddress, logData.bytes);
static constexpr uint32_t kAddressLogData = offsetof(NonVolatileDataAddress, logData.dummyLogData);
static constexpr uint32_t kCapacityLogData = Fram::kMaxAddress - kAddressLogData;

/* ラインセンサー・マーカーセンサーのキャリブレーション情報を書き込み */
bool WriteLineSensorCalibrationData(const std::array<uint16_t, 16>& lineMin, const std::array<uint16_t, 16>& lineMax,
                                    const std::array<float, 16>& lineCoeff, const std::array<uint16_t, 2>& markerMax);
/* ラインセンサー・マーカーセンサーのキャリブレーション情報を読み出し */
bool ReadLineSensorCalibrationData(std::array<uint16_t, 16>& lineMin, std::array<uint16_t, 16>& lineMax,
                                   std::array<float, 16>& lineCoeff, std::array<uint16_t, 2>& markerMax);

/* 曲率を書き込み */
bool WriteVelocityMappingData(const std::array<float, kMappingMaxPoints>& deltaDistanceArray,
                              const std::array<float, kMappingMaxPoints>& deltaAngleArray, uint16_t numPoints);
/* 曲率を読み出し */
bool ReadVelocityMappingData(std::array<float, kMappingMaxPoints>& deltaDistanceArray,
                             std::array<float, kMappingMaxPoints>& deltaAngleArray, uint16_t& numPoints);
/* 補正位置を書き込み */
bool WritePositionCorrectionData(const std::array<float, kCorrectionMaxPoints>& crossLineArray,
                                 uint16_t numCrossLinePoints,
                                 const std::array<float, kCorrectionMaxPoints>& curveMarkerArray,
                                 uint16_t numCurveMarkerPoints);
/* 補正位置を読み出し */
bool ReadPositionCorrectionData(std::array<float, kCorrectionMaxPoints>& crossLineArray, uint16_t& numCrossLinePoints,
                                std::array<float, kCorrectionMaxPoints>& curveMarkerArray,
                                uint16_t& numCurveMarkerPoints);

/* ログ数を書き込み */
bool WriteLogDataNumBytes(uint32_t bytes);
/* ログ数を読み出し */
bool ReadLogDataNumBytes(uint32_t& bytes);

}  // namespace NonVolatileData

#endif  // APP_NON_VOLATILE_DATA_H_
