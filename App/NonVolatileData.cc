
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
bool WriteVelocityMappingData(const std::vector<float>& deltaDistanceVec, const std::vector<float>& accYawRateVec) {
  auto& fram = Fram::Instance();
  uint16_t deltaDistanceNum = static_cast<uint16_t>(deltaDistanceVec.size());
  uint16_t accYawRateNum = static_cast<uint16_t>(accYawRateVec.size());

  /* 書き込み可能なサイズかチェック */
  if (kCapacityVelocityMapping < deltaDistanceNum || kCapacityVelocityMapping < accYawRateNum ||
      deltaDistanceNum != accYawRateNum) {
    return false;
  }

  /* サイズを書き込み */
  if (!fram.Write(kAddressVelocityMappingDataNum, &deltaDistanceNum, sizeof(uint16_t))) {
    return false;
  }

  /* データを書き込み(浮動小数点を16ビット整数に変換) */
  for (uint32_t num = 0; num < deltaDistanceNum; num++) {
    if (deltaDistanceVec[num] < 0.0f || (static_cast<float>(UINT16_MAX) / 1000.0f) < deltaDistanceVec[num] ||
        accYawRateVec[num] < 0.0f || (static_cast<float>(UINT16_MAX) / 1000.0f) < accYawRateVec[num]) {
      return false;
    }
    uint16_t deltaDistance = static_cast<uint16_t>(deltaDistanceVec[num] * 1000.0f);
    uint16_t accYawRate = static_cast<uint16_t>(accYawRateVec[num] * 1000.0f);
    if (!fram.Write(kAddressVelocityMappingDataDeltaDistance + num * sizeof(uint16_t), &deltaDistance,
                    sizeof(uint16_t)) ||
        !fram.Write(kAddressVelocityMappingDataAccYaw + num * sizeof(uint16_t), &accYawRate, sizeof(uint16_t))) {
      return false;
    }
  }

  return true;
}
/* 曲率を読み出し */
bool ReadVelocityMappingData(std::vector<float>& deltaDistanceVec, std::vector<float>& accYawRateVec) {
  auto& fram = Fram::Instance();
  uint16_t totalNum = 0;

  /* サイズを読み出し */
  if (!fram.Read(kAddressVelocityMappingDataNum, &totalNum, sizeof(uint16_t))) {
    return false;
  }
  if (kCapacityVelocityMapping < totalNum) {
    return false;
  }

  /* データを書き込み(浮動小数点を16ビット整数に変換) */
  for (uint32_t num = 0; num < totalNum; num++) {
    uint16_t deltaDistance = 0;
    uint16_t accYawRate = 0;
    if (!fram.Read(kAddressVelocityMappingDataDeltaDistance + num * sizeof(uint16_t), &deltaDistance,
                   sizeof(uint16_t)) ||
        !fram.Read(kAddressVelocityMappingDataAccYaw + num * sizeof(uint16_t), &accYawRate, sizeof(uint16_t))) {
      return false;
    }
    deltaDistanceVec.push_back(static_cast<float>(deltaDistance) / 1000.0f);
    accYawRateVec.push_back(static_cast<float>(accYawRate) / 1000.0f);
  }

  return true;
}
/* 補正位置を書き込み */
bool WritePositionCorrectionData(const std::vector<float>& crossLineVec, const std::vector<float>& curveMarkerVec) {
  auto& fram = Fram::Instance();
  uint16_t crossLineNum = static_cast<uint16_t>(crossLineVec.size());
  uint16_t curveMarkerNum = static_cast<uint16_t>(curveMarkerVec.size());

  /* クロスラインの位置を書き込み */
  {
    if (kCapacityPositionCorrection < crossLineNum) {
      return false;
    }
    if (!fram.Write(kAddressPositionCorrectionDataCrossLineNum, &crossLineNum, sizeof(uint16_t))) {
      return false;
    }
    for (uint32_t num = 0; num < crossLineNum; num++) {
      if (crossLineVec[num] < 0.0f || (static_cast<float>(UINT16_MAX) / 1000.0f) < crossLineVec[num]) {
        return false;
      }
      uint16_t crossLine = static_cast<uint16_t>(crossLineVec[num] * 1000.0f);
      if (!fram.Write(kAddressPositionCorrectionDataCrossLine + num * sizeof(uint16_t), &crossLine, sizeof(uint16_t))) {
        return false;
      }
    }
  }

  /* マーカーの位置を書き込み */
  {
    if (kCapacityPositionCorrection < curveMarkerNum) {
      return false;
    }
    if (!fram.Write(kAddressPositionCorrectionDataCurveMarkerNum, &curveMarkerNum, sizeof(uint16_t))) {
      return false;
    }
    for (uint32_t num = 0; num < curveMarkerNum; num++) {
      if (curveMarkerVec[num] < 0.0f || (static_cast<float>(UINT16_MAX) / 1000.0f) < curveMarkerVec[num]) {
        return false;
      }
      uint16_t curveMarker = static_cast<uint16_t>(curveMarkerVec[num] * 1000.0f);
      if (!fram.Write(kAddressPositionCorrectionDataCurveMarker + num * sizeof(uint16_t), &curveMarker,
                      sizeof(uint16_t))) {
        return false;
      }
    }
  }

  return true;
}
/* 補正位置を読み出し */
bool ReadPositionCorrectionData(std::vector<float>& crossLineVec, std::vector<float>& curveMarkerVec) {
  auto& fram = Fram::Instance();

  /* クロスラインの位置を読み出し */
  {
    uint16_t crossLineNum = 0;
    if (!fram.Read(kAddressPositionCorrectionDataCrossLineNum, &crossLineNum, sizeof(uint16_t))) {
      return false;
    }
    if (kCapacityPositionCorrection < crossLineNum) {
      return false;
    }
    for (uint32_t num = 0; num < crossLineNum; num++) {
      uint16_t crossLine = 0;
      if (!fram.Read(kAddressPositionCorrectionDataCrossLine + num * sizeof(uint16_t), &crossLine, sizeof(uint16_t))) {
        return false;
      }
      crossLineVec.push_back(static_cast<float>(crossLine) / 1000.0f);
    }
  }

  /* マーカーの位置を読み出し */
  {
    uint16_t curveMarkerNum = 0;
    if (!fram.Read(kAddressPositionCorrectionDataCurveMarkerNum, &curveMarkerNum, sizeof(uint16_t))) {
      return false;
    }
    if (kCapacityPositionCorrection < curveMarkerNum) {
      return false;
    }
    for (uint32_t num = 0; num < curveMarkerNum; num++) {
      uint16_t curveMarker = 0;
      if (!fram.Read(kAddressPositionCorrectionDataCurveMarker + num * sizeof(uint16_t), &curveMarker,
                     sizeof(uint16_t))) {
        return false;
      }
      curveMarkerVec.push_back(static_cast<float>(curveMarker) / 1000.0f);
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
