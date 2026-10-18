#pragma once

// 算法适配器内部的强类型存储，不安装到 C SDK。
typedef struct MvCalibDataImpl* MvCalibData;
typedef struct MvDictImpl* MvDict;
typedef struct MvNCCModelImpl* MvNCCModel;
typedef struct MvCameraSetupModelImpl* MvCameraSetupModel;
typedef struct MvObjectModel3DImpl* MvObjectModel3D;
typedef struct MvShapeModelImpl* MvShapeModel;
typedef struct MvStructuredLightModelImpl* MvStructuredLightModel;

typedef int32_t MvDictValueKind;

#define MV_DICT_VALUE_TUPLE ((int32_t)0)
#define MV_DICT_VALUE_ICONIC_TUPLE ((int32_t)1)
