
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

  /* データを書き込み(浮動小数点を16ビット整数に変換) */
  for (uint32_t num = 0; num < numPoints; num++) {
    if (deltaDistanceArray[num] < 0.0f || (static_cast<float>(UINT16_MAX) / 1000.0f) < deltaDistanceArray[num] ||
        deltaAngleArray[num] < 0.0f || (static_cast<float>(UINT16_MAX) / 1000.0f) < deltaAngleArray[num]) {
      return false;
    }
    uint16_t deltaDistance = static_cast<uint16_t>(deltaDistanceArray[num] * 1000.0f);
    uint16_t deltaAngle = static_cast<uint16_t>(deltaAngleArray[num] * 1000.0f);
    if (!fram.Write(kAddressVelocityMappingDataDeltaDistance + num * sizeof(uint16_t), &deltaDistance,
                    sizeof(uint16_t)) ||
        !fram.Write(kAddressVelocityMappingDataDeltaAngle + num * sizeof(uint16_t), &deltaAngle, sizeof(uint16_t))) {
      return false;
    }
  }

  return true;
}
/* 曲率を読み出し */
bool ReadVelocityMappingData(std::array<float, kMappingMaxPoints>& deltaDistanceArray,
                             std::array<float, kMappingMaxPoints>& deltaAngleArray, uint16_t& numPoints) {
  auto& fram = Fram::Instance();
  uint16_t numPoints_ = 0;

  /* サイズを読み出し */
  if (!fram.Read(kAddressVelocityMappingDataNumPoints, &numPoints_, sizeof(uint16_t))) {
    return false;
  }
  if (kMappingMaxPoints < numPoints_) {
    return false;
  }

  /* データを書き込み(浮動小数点を16ビット整数に変換) */
  for (uint32_t num = 0; num < numPoints_; num++) {
    uint16_t deltaDistance = 0;
    uint16_t deltaAngle = 0;
    if (!fram.Read(kAddressVelocityMappingDataDeltaDistance + num * sizeof(uint16_t), &deltaDistance,
                   sizeof(uint16_t)) ||
        !fram.Read(kAddressVelocityMappingDataDeltaAngle + num * sizeof(uint16_t), &deltaAngle, sizeof(uint16_t))) {
      return false;
    }
    deltaDistanceArray[num] = static_cast<float>(deltaDistance) / 1000.0f;
    deltaAngleArray[num] = static_cast<float>(deltaAngle) / 1000.0f;
  }
  numPoints = numPoints_;

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
    for (uint32_t num = 0; num < numCrossLinePoints; num++) {
      if (crossLineArray[num] < 0.0f || (static_cast<float>(UINT16_MAX) / 1000.0f) < crossLineArray[num]) {
        return false;
      }
      uint16_t crossLine = static_cast<uint16_t>(crossLineArray[num] * 1000.0f);
      if (!fram.Write(kAddressPositionCorrectionDataCrossLine + num * sizeof(uint16_t), &crossLine, sizeof(uint16_t))) {
        return false;
      }
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
    for (uint32_t num = 0; num < numCurveMarkerPoints; num++) {
      if (curveMarkerArray[num] < 0.0f || (static_cast<float>(UINT16_MAX) / 1000.0f) < curveMarkerArray[num]) {
        return false;
      }
      uint16_t curveMarker = static_cast<uint16_t>(curveMarkerArray[num] * 1000.0f);
      if (!fram.Write(kAddressPositionCorrectionDataCurveMarker + num * sizeof(uint16_t), &curveMarker,
                      sizeof(uint16_t))) {
        return false;
      }
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
    uint16_t numCrossLinePoints_ = 0;
    if (!fram.Read(kAddressPositionCorrectionDataNumCrossLinePoints, &numCrossLinePoints_, sizeof(uint16_t))) {
      return false;
    }
    if (kCorrectionMaxPoints < numCrossLinePoints_) {
      return false;
    }
    for (uint32_t num = 0; num < numCrossLinePoints_; num++) {
      uint16_t crossLine = 0;
      if (!fram.Read(kAddressPositionCorrectionDataCrossLine + num * sizeof(uint16_t), &crossLine, sizeof(uint16_t))) {
        return false;
      }
      crossLineArray[num] = static_cast<float>(crossLine) / 1000.0f;
    }
    numCrossLinePoints = numCrossLinePoints_;
  }

  /* マーカーの位置を読み出し */
  {
    uint16_t numCurveMarkerPoints_ = 0;
    if (!fram.Read(kAddressPositionCorrectionDataNumCurveMarkerPoints, &numCurveMarkerPoints_, sizeof(uint16_t))) {
      return false;
    }
    if (kCorrectionMaxPoints < numCurveMarkerPoints_) {
      return false;
    }
    for (uint32_t num = 0; num < numCurveMarkerPoints_; num++) {
      uint16_t curveMarker = 0;
      if (!fram.Read(kAddressPositionCorrectionDataCurveMarker + num * sizeof(uint16_t), &curveMarker,
                     sizeof(uint16_t))) {
        return false;
      }
      curveMarkerArray[num] = static_cast<float>(curveMarker) / 1000.0f;
    }
    numCurveMarkerPoints = numCurveMarkerPoints_;
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
