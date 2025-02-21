
#include "NonVolatileData.h"

namespace NonVolatileData {
/* ラインセンサー・マーカーセンサーのキャリブレーション情報を書き込み */
bool WriteLineSensorCalibrationData(const std::array<uint16_t, 16>& lineMin, const std::array<uint16_t, 16>& lineMax,
                                    const std::array<float, 16>& lineCoeff, const std::array<uint16_t, 2>& markerMax) {
  auto& fram = Fram::Instance();

  return fram.Write(kAddressSensorCalibrationDataLineMin, &lineMin, sizeof(lineMin)) &&
         fram.Write(kAddressSensorCalibrationDataLineMax, &lineMax, sizeof(lineMax)) &&
         fram.Write(kAddressSensorCalibrationDataLineCoeff, &lineCoeff, sizeof(lineCoeff)) &&
         fram.Write(kAddressSensorCalibrationDataMarkerMax, &markerMax, sizeof(markerMax));
}
/* ラインセンサー・マーカーセンサーのキャリブレーション情報を読み出し */
bool ReadLineSensorCalibrationData(std::array<uint16_t, 16>& lineMin, std::array<uint16_t, 16>& lineMax,
                                   std::array<float, 16>& lineCoeff, std::array<uint16_t, 2>& markerMax) {
  auto& fram = Fram::Instance();

  return fram.Read(kAddressSensorCalibrationDataLineMin, &lineMin, sizeof(lineMin)) &&
         fram.Read(kAddressSensorCalibrationDataLineMax, &lineMax, sizeof(lineMax)) &&
         fram.Read(kAddressSensorCalibrationDataLineCoeff, &lineCoeff, sizeof(lineCoeff)) &&
         fram.Read(kAddressSensorCalibrationDataMarkerMax, &markerMax, sizeof(markerMax));
}
/* 曲率を書き込み */
bool WriteVelocityMappingData(const std::array<float, kMappingMaxPoints>& deltaDistanceArray,
                              const std::array<float, kMappingMaxPoints>& deltaAngleArray, uint16_t numPoints) {
  auto& fram = Fram::Instance();
  /* 書き込み可能なサイズかチェック */
  if (kMappingMaxPoints < numPoints) {
    return false;
  }

  /* サイズを書き込み */
  if (!fram.Write(kAddressVelocityMappingDataNumPoints, &numPoints, sizeof(uint16_t))) {
    return false;
  }
  /* データを書き込み */
  if (!fram.Write(kAddressVelocityMappingDataDeltaDistance, &deltaDistanceArray, sizeof(float) * numPoints) ||
      !fram.Write(kAddressVelocityMappingDataDeltaAngle, &deltaAngleArray, sizeof(float) * numPoints)) {
    return false;
  }

  return true;
}
/* 曲率を読み出し */
bool ReadVelocityMappingData(std::array<float, kMappingMaxPoints>& deltaDistanceArray,
                             std::array<float, kMappingMaxPoints>& deltaAngleArray, uint16_t& numPoints) {
  auto& fram = Fram::Instance();

  /* サイズを読み出し */
  if (!fram.Read(kAddressVelocityMappingDataNumPoints, &numPoints, sizeof(uint16_t))) {
    return false;
  }
  if (kMappingMaxPoints < numPoints) {
    return false;
  }

  /* データを読み出し */
  if (!fram.Read(kAddressVelocityMappingDataDeltaDistance, &deltaDistanceArray, sizeof(float) * numPoints) ||
      !fram.Read(kAddressVelocityMappingDataDeltaAngle, &deltaAngleArray, sizeof(float) * numPoints)) {
    return false;
  }

  return true;
}
/* 補正位置を書き込み */
bool WritePositionCorrectionData(const std::array<float, kCorrectionMaxPoints>& crossLineArray,
                                 uint16_t numCrossLinePoints,
                                 const std::array<float, kCorrectionMaxPoints>& curveMarkerArray,
                                 uint16_t numCurveMarkerPoints) {
  auto& fram = Fram::Instance();

  /* クロスラインの位置を書き込み */
  {
    if (kCorrectionMaxPoints < numCrossLinePoints) {
      return false;
    }
    if (!fram.Write(kAddressPositionCorrectionDataNumCrossLinePoints, &numCrossLinePoints, sizeof(uint16_t))) {
      return false;
    }
    if (!fram.Write(kAddressPositionCorrectionDataCrossLine, &crossLineArray, sizeof(float) * numCrossLinePoints)) {
      return false;
    }
  }

  /* マーカーの位置を書き込み */
  {
    if (kCorrectionMaxPoints < numCurveMarkerPoints) {
      return false;
    }
    if (!fram.Write(kAddressPositionCorrectionDataNumCurveMarkerPoints, &numCurveMarkerPoints, sizeof(uint16_t))) {
      return false;
    }
    if (!fram.Write(kAddressPositionCorrectionDataCurveMarker, &curveMarkerArray,
                    sizeof(float) * numCurveMarkerPoints)) {
      return false;
    }
  }

  return true;
}
/* 補正位置を読み出し */
bool ReadPositionCorrectionData(std::array<float, kCorrectionMaxPoints>& crossLineArray, uint16_t& numCrossLinePoints,
                                std::array<float, kCorrectionMaxPoints>& curveMarkerArray,
                                uint16_t& numCurveMarkerPoints) {
  auto& fram = Fram::Instance();

  /* クロスラインの位置を読み出し */
  {
    if (!fram.Read(kAddressPositionCorrectionDataNumCrossLinePoints, &numCrossLinePoints, sizeof(uint16_t))) {
      return false;
    }
    if (kCorrectionMaxPoints < numCrossLinePoints) {
      return false;
    }
    if (!fram.Read(kAddressPositionCorrectionDataCrossLine, &crossLineArray, sizeof(float) * numCrossLinePoints)) {
      return false;
    }
  }

  /* マーカーの位置を読み出し */
  {
    if (!fram.Read(kAddressPositionCorrectionDataNumCurveMarkerPoints, &numCurveMarkerPoints, sizeof(uint16_t))) {
      return false;
    }
    if (kCorrectionMaxPoints < numCurveMarkerPoints) {
      return false;
    }
    if (!fram.Read(kAddressPositionCorrectionDataCurveMarker, &curveMarkerArray,
                   sizeof(float) * numCurveMarkerPoints)) {
      return false;
    }
  }

  return true;
}
/* ログ数を書き込み */
bool WriteLogDataNumBytes(uint32_t bytes) {
  auto& fram = Fram::Instance();
  bytes = std::min(bytes, kCapacityLogData);
  if (!fram.Write(kAddressLogDataBytes, &bytes, sizeof(uint32_t))) {
    return false;
  }
  return true;
}
/* ログ数を読み出し */
bool ReadLogDataNumBytes(uint32_t& bytes) {
  auto& fram = Fram::Instance();
  uint32_t b = 0;
  if (!fram.Read(kAddressLogDataBytes, &b, sizeof(uint32_t))) {
    return false;
  }
  bytes = std::min(b, kCapacityLogData);
  return true;
}
}  // namespace NonVolatileData
