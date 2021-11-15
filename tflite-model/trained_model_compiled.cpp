/* Generated by Edge Impulse
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
// Generated on: 14.11.2021 23:32:24

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "edge-impulse-sdk/tensorflow/lite/c/builtin_op_data.h"
#include "edge-impulse-sdk/tensorflow/lite/c/common.h"
#include "edge-impulse-sdk/tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#if EI_CLASSIFIER_PRINT_STATE
#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C" {
    extern void ei_printf(const char *format, ...);
}
#else
extern void ei_printf(const char *format, ...);
#endif
#endif

#if defined __GNUC__
#define ALIGN(X) __attribute__((aligned(X)))
#elif defined _MSC_VER
#define ALIGN(X) __declspec(align(X))
#elif defined __TASKING__
#define ALIGN(X) __align(X)
#endif

using namespace tflite;
using namespace tflite::ops;
using namespace tflite::ops::micro;

namespace {

constexpr int kTensorArenaSize = 272;

#if defined(EI_CLASSIFIER_ALLOCATION_STATIC)
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16);
#elif defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX)
#pragma Bss(".tensor_arena")
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16);
#pragma Bss()
#elif defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX_GNU)
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16) __attribute__((section(".tensor_arena")));
#else
#define EI_CLASSIFIER_ALLOCATION_HEAP 1
uint8_t* tensor_arena = NULL;
#endif

static uint8_t* tensor_boundary;
static uint8_t* current_location;

template <int SZ, class T> struct TfArray {
  int sz; T elem[SZ];
};
enum used_operators_e {
  OP_FULLY_CONNECTED, OP_SOFTMAX,  OP_LAST
};
struct TensorInfo_t { // subset of TfLiteTensor used for initialization from constant memory
  TfLiteAllocationType allocation_type;
  TfLiteType type;
  void* data;
  TfLiteIntArray* dims;
  size_t bytes;
  TfLiteQuantization quantization;
};
struct NodeInfo_t { // subset of TfLiteNode used for initialization from constant memory
  struct TfLiteIntArray* inputs;
  struct TfLiteIntArray* outputs;
  void* builtin_data;
  used_operators_e used_op_index;
};

TfLiteContext ctx{};
TfLiteTensor tflTensors[11];
TfLiteEvalTensor tflEvalTensors[11];
TfLiteRegistration registrations[OP_LAST];
TfLiteNode tflNodes[4];

const TfArray<2, int> tensor_dimension0 = { 2, { 1,33 } };
const TfArray<1, float> quant0_scale = { 1, { 0.016495488584041595, } };
const TfArray<1, int> quant0_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant0 = { (TfLiteFloatArray*)&quant0_scale, (TfLiteIntArray*)&quant0_zero, 0 };
const ALIGN(8) int32_t tensor_data1[30] = { -1, 657, -15, 11, -23, 1053, 1822, -643, 4198, 1280, 3754, 3048, -47, -28, -659, 1678, 3623, -83, 64, -21, -190, 3246, 0, 849, -18, 362, 446, 35, -40, 3018, };
const TfArray<1, int> tensor_dimension1 = { 1, { 30 } };
const TfArray<1, float> quant1_scale = { 1, { 0.00014499638928100467, } };
const TfArray<1, int> quant1_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant1 = { (TfLiteFloatArray*)&quant1_scale, (TfLiteIntArray*)&quant1_zero, 0 };
const ALIGN(8) int32_t tensor_data2[10] = { -187, -921, -336, 2398, -1217, -1055, 328, 3888, -951, -584, };
const TfArray<1, int> tensor_dimension2 = { 1, { 10 } };
const TfArray<1, float> quant2_scale = { 1, { 0.000160534618771635, } };
const TfArray<1, int> quant2_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant2 = { (TfLiteFloatArray*)&quant2_scale, (TfLiteIntArray*)&quant2_zero, 0 };
const ALIGN(8) int32_t tensor_data3[3] = { 455, -338, -398, };
const TfArray<1, int> tensor_dimension3 = { 1, { 3 } };
const TfArray<1, float> quant3_scale = { 1, { 0.00095447729108855128, } };
const TfArray<1, int> quant3_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant3 = { (TfLiteFloatArray*)&quant3_scale, (TfLiteIntArray*)&quant3_zero, 0 };
const ALIGN(8) int8_t tensor_data4[30*33] = { 
  -34, -40, -7, 21, -40, -5, 1, -8, -34, 18, 25, -24, 16, -10, -35, 18, -20, 11, 7, 5, -29, -12, 43, 28, -21, -3, -9, -19, 18, 29, 9, 6, 18, 
  39, 35, 3, 17, 36, 30, 28, 12, 12, -5, 21, 36, 39, 38, -9, 9, 11, -24, 28, -9, 41, 39, 18, 36, -10, 23, -9, -10, 11, -24, -29, -34, 7, 
  49, 30, 34, 40, -19, -9, 9, 22, -2, 52, 16, 29, 24, -21, -13, 24, -16, -4, -33, 12, -36, 2, -3, -39, -8, -15, -33, -14, -7, 11, -12, 4, 11, 
  -12, 29, 34, 16, -12, -2, 35, 51, 24, 6, 10, -11, 47, 34, -17, 20, -5, 28, -16, 26, 0, 49, -50, 30, -14, -25, -2, -11, -8, -28, -44, -44, 1, 
  45, -19, 39, 30, -13, 27, 16, -8, 18, 11, 41, 25, 22, -11, -11, -16, -4, 1, 20, -5, 13, 29, 24, 33, -2, -7, -24, 10, -13, -44, -18, 3, 13, 
  -8, 39, -23, -19, -2, -23, -34, -41, 10, -33, -31, 20, 43, 47, -18, 22, -24, -10, 8, 27, 38, -19, 68, 46, 20, 38, -1, 40, 40, 21, 47, -7, 42, 
  -12, 45, -1, 13, 24, -26, 11, 11, -25, -22, 12, 51, 29, 19, 7, -5, 9, 13, 28, 58, 37, 11, 39, 33, 34, 6, 6, 20, -20, 4, 28, 46, 69, 
  -22, -9, -18, 20, -5, 20, -32, -7, -35, -21, -13, -8, -7, 18, -28, 20, -6, -4, 1, 11, -40, -3, -28, -37, -41, 6, 12, -27, 22, 19, 18, 13, -1, 
  25, 9, -16, 29, -3, -26, -29, -11, 21, 13, -2, -10, 34, 0, -1, -1, -15, -29, 12, 9, -9, 24, -16, 14, -17, -13, -1, -14, -26, -30, -15, -45, -69, 
  40, 29, 32, 16, 6, -27, -3, 11, 3, 10, 31, 41, 37, -5, -20, 33, 27, 32, -28, -35, -28, 5, -36, 13, -33, -25, 17, -27, 18, -14, -59, -39, -35, 
  -84, 0, -16, -1, 5, -28, 23, -52, -34, -24, -68, -26, 7, 51, 14, -14, -15, -32, 38, 6, 53, -1, 13, 12, 10, 22, 8, 40, 34, -13, 40, 36, 6, 
  48, 18, 10, 45, 21, -22, 38, -1, 20, -2, 42, -18, 37, 5, -3, -8, 24, 9, 34, 7, -25, -15, -52, 32, 9, 13, -26, 7, -33, -49, -25, -24, -57, 
  84, -16, -3, 20, 22, -52, 25, 22, 5, 47, 25, 56, -28, 18, -14, 1, -29, 4, 6, -21, 24, 5, 49, 8, -41, -16, 39, -17, 5, -18, -31, -29, 1, 
  35, -19, 33, 18, 25, 10, 31, -16, 30, 15, 38, 22, 18, -28, -11, -1, -34, 2, 12, 14, -27, 23, 44, 32, 27, -22, -25, 0, 29, -18, 7, -25, 20, 
  -27, -19, -22, 27, 12, 7, -13, -6, -35, -35, -9, -24, -24, -3, 14, 7, 20, 30, -11, -8, -2, -19, 19, -21, 7, -28, -14, -28, 24, -36, -2, 25, 22, 
  -57, 3, -2, 2, -32, -6, 22, -39, -32, 5, -68, 25, 36, 2, -1, 2, -16, 6, 31, 31, -14, 9, 46, 41, 11, 49, 41, 18, 36, 9, 40, 41, 22, 
  -93, -12, -1, 4, -9, -16, 11, -62, -57, -1, -28, -30, -40, -4, -31, -26, 22, -17, -7, -24, -13, -48, -9, -25, -12, -5, 6, -4, -25, -19, -2, 17, -3, 
  0, -10, -15, 47, 21, 33, 8, -3, 23, -4, 4, -22, -8, -7, -6, -3, 31, 24, 34, 24, 21, -22, -2, 18, 20, 7, 1, -31, -26, -35, 19, 14, -7, 
  -28, 5, 20, 6, -11, 9, -14, 23, 15, -45, 18, 15, -27, 3, 1, 0, -22, -30, -1, 13, 29, -12, 82, 25, 54, -127, -12, -29, 10, 51, 94, 84, 96, 
  23, 18, 30, 9, 10, 23, 37, 4, 13, 18, 43, 56, 11, 14, 14, 25, 24, -12, 21, 9, -41, 18, 12, 18, 14, -13, -19, -20, 28, -1, -6, -16, -21, 
  -33, -38, -12, -12, -25, 5, 31, 23, 23, -11, 27, 12, -30, -27, 25, -38, -20, -25, -20, -37, -39, -39, 1, 2, 14, -28, 18, -1, 15, 4, -16, 12, -3, 
  -23, 4, -4, 26, 20, 21, 13, 16, -44, -38, -46, 33, 30, 35, -17, 18, -20, -7, -1, 39, 49, -14, -10, -11, 37, 42, -3, -4, 8, -12, 27, 18, 10, 
  -34, 6, 0, -11, -8, -5, -12, -17, -8, 31, -24, -2, -33, -32, -17, -6, 29, -26, -6, -15, 26, -11, -23, -19, -5, -26, 28, 8, -3, -3, 24, -15, -27, 
  -29, 2, -2, -17, -4, 5, -5, 14, -2, -16, -36, 10, 43, -7, 49, -1, 32, 19, 52, 11, 51, 27, 33, 44, 20, 38, 21, 7, 28, 9, 55, 27, 71, 
  80, 48, 28, 1, 0, 8, 16, 25, 21, 28, 36, 35, 17, 11, -15, 13, -16, -26, 31, 6, -12, 59, -16, -25, -10, -35, 5, -27, -9, -38, -1, -66, -20, 
  31, 6, 18, 44, 26, 1, -11, 41, 8, 60, 33, 53, 35, 5, 1, -19, 16, 21, 11, -5, 26, -3, -24, -13, -12, -10, -10, -20, 19, -33, -54, 8, -31, 
  85, 35, 7, 13, 5, 1, -23, -1, 2, 4, 43, 58, -4, -24, 19, 14, 1, -7, -6, 19, -18, 22, -30, 5, -4, -39, 16, 25, -27, -8, -1, -2, -4, 
  101, -68, -65, -31, -35, -20, 22, 42, 22, 28, 45, 59, -22, -49, -48, -32, -30, -25, 17, 41, 51, 29, 43, -57, -18, -27, 7, 26, -19, 53, 32, 40, 27, 
  50, 41, 34, 26, 5, -11, -35, -9, 0, -2, 34, 40, 19, 44, 2, 42, 31, 26, 47, 21, 39, 55, 58, 43, 12, 10, 23, -26, 30, 43, 37, 25, 71, 
  55, 39, -22, -1, -12, 34, 5, 46, 13, 39, 47, -2, -12, 5, 11, -17, -26, -1, 46, -17, 28, 56, 16, 22, 41, 32, -10, -5, 29, -6, 37, 39, 49, 
};
const TfArray<2, int> tensor_dimension4 = { 2, { 30,33 } };
const TfArray<1, float> quant4_scale = { 1, { 0.0087900636717677116, } };
const TfArray<1, int> quant4_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant4 = { (TfLiteFloatArray*)&quant4_scale, (TfLiteIntArray*)&quant4_zero, 0 };
const ALIGN(8) int8_t tensor_data5[10*30] = { 
  -26, -5, 8, 7, 34, -28, -14, 7, -38, -38, -13, -26, 35, 32, -15, -3, -37, 6, -12, 2, 27, -19, -34, -11, -20, -35, 34, -23, -35, -21, 
  38, 83, 69, -6, 106, 38, 49, -14, -13, 13, -16, -19, 73, 73, -23, -26, -63, 58, 91, 51, 14, 24, 1, 38, 73, 114, 66, 127, 85, 49, 
  -40, 52, 34, -1, 50, 8, 21, -11, 52, 63, -62, 3, 73, -18, 19, -32, -51, 48, -72, 9, 27, -19, -2, 4, 64, 38, 17, 53, 21, 51, 
  -12, -1, -50, -55, -6, 43, 73, 22, 80, -60, 110, 28, -21, -34, -14, 69, 96, 23, 28, -19, -13, 68, 29, 72, -46, -51, -83, -21, 19, 49, 
  25, 23, -31, -57, -20, 56, 18, -13, -26, 4, 13, -37, 6, 45, 0, 28, -22, -23, 113, 25, 35, 46, 16, 61, 9, -42, -21, 5, 39, 19, 
  43, 54, -40, -19, -11, 13, 35, -20, 35, -68, -29, 41, 11, 41, -15, 52, -63, 8, 33, 44, 36, 27, -6, 28, -49, -17, -55, 16, -16, -5, 
  22, -18, 5, 49, 19, -21, 3, 18, 46, 67, 10, 25, -44, -7, -8, -13, 69, -16, -29, -27, 29, 16, -30, 15, -12, 38, 19, -19, 25, 21, 
  -62, 44, -2, -15, -12, -19, 2, -22, 114, 69, 78, 90, -11, -4, 0, -26, 69, 1, -61, 10, 24, 75, 32, -48, -9, 48, 50, 1, -36, 58, 
  -21, 45, 19, 36, 58, 10, 31, -30, 1, -12, -8, 44, 94, 60, 3, 24, -70, 51, 3, 18, -40, 22, 14, -1, 58, 4, 26, 104, 49, 14, 
  -19, 8, 25, 51, -35, 11, 11, -13, 35, 2, -31, 16, -41, -6, -30, -5, 26, 31, -8, 29, 7, 0, -29, 24, -2, 16, 5, -36, 6, -54, 
};
const TfArray<2, int> tensor_dimension5 = { 2, { 10,30 } };
const TfArray<1, float> quant5_scale = { 1, { 0.0090477494522929192, } };
const TfArray<1, int> quant5_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant5 = { (TfLiteFloatArray*)&quant5_scale, (TfLiteIntArray*)&quant5_zero, 0 };
const ALIGN(8) int8_t tensor_data6[3*10] = { 
  -18, -127, -89, 73, 12, -13, 19, 63, -54, -13, 
  28, 17, 71, -67, 4, -17, 8, 27, 46, 1, 
  -50, 29, 0, 24, 49, 28, -26, -62, 19, -18, 
};
const TfArray<2, int> tensor_dimension6 = { 2, { 3,10 } };
const TfArray<1, float> quant6_scale = { 1, { 0.012856373563408852, } };
const TfArray<1, int> quant6_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant6 = { (TfLiteFloatArray*)&quant6_scale, (TfLiteIntArray*)&quant6_zero, 0 };
const TfArray<2, int> tensor_dimension7 = { 2, { 1,30 } };
const TfArray<1, float> quant7_scale = { 1, { 0.017743043601512909, } };
const TfArray<1, int> quant7_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant7 = { (TfLiteFloatArray*)&quant7_scale, (TfLiteIntArray*)&quant7_zero, 0 };
const TfArray<2, int> tensor_dimension8 = { 2, { 1,10 } };
const TfArray<1, float> quant8_scale = { 1, { 0.074241563677787781, } };
const TfArray<1, int> quant8_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant8 = { (TfLiteFloatArray*)&quant8_scale, (TfLiteIntArray*)&quant8_zero, 0 };
const TfArray<2, int> tensor_dimension9 = { 2, { 1,3 } };
const TfArray<1, float> quant9_scale = { 1, { 0.26395535469055176, } };
const TfArray<1, int> quant9_zero = { 1, { 33 } };
const TfLiteAffineQuantization quant9 = { (TfLiteFloatArray*)&quant9_scale, (TfLiteIntArray*)&quant9_zero, 0 };
const TfArray<2, int> tensor_dimension10 = { 2, { 1,3 } };
const TfArray<1, float> quant10_scale = { 1, { 0.00390625, } };
const TfArray<1, int> quant10_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant10 = { (TfLiteFloatArray*)&quant10_scale, (TfLiteIntArray*)&quant10_zero, 0 };
const TfLiteFullyConnectedParams opdata0 = { kTfLiteActRelu, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs0 = { 3, { 0,4,1 } };
const TfArray<1, int> outputs0 = { 1, { 7 } };
const TfLiteFullyConnectedParams opdata1 = { kTfLiteActRelu, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs1 = { 3, { 7,5,2 } };
const TfArray<1, int> outputs1 = { 1, { 8 } };
const TfLiteFullyConnectedParams opdata2 = { kTfLiteActNone, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs2 = { 3, { 8,6,3 } };
const TfArray<1, int> outputs2 = { 1, { 9 } };
const TfLiteSoftmaxParams opdata3 = { 1 };
const TfArray<1, int> inputs3 = { 1, { 9 } };
const TfArray<1, int> outputs3 = { 1, { 10 } };
const TensorInfo_t tensorData[] = {
  { kTfLiteArenaRw, kTfLiteInt8, tensor_arena + 0, (TfLiteIntArray*)&tensor_dimension0, 33, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant0))}, },
  { kTfLiteMmapRo, kTfLiteInt32, (void*)tensor_data1, (TfLiteIntArray*)&tensor_dimension1, 120, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant1))}, },
  { kTfLiteMmapRo, kTfLiteInt32, (void*)tensor_data2, (TfLiteIntArray*)&tensor_dimension2, 40, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant2))}, },
  { kTfLiteMmapRo, kTfLiteInt32, (void*)tensor_data3, (TfLiteIntArray*)&tensor_dimension3, 12, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant3))}, },
  { kTfLiteMmapRo, kTfLiteInt8, (void*)tensor_data4, (TfLiteIntArray*)&tensor_dimension4, 990, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant4))}, },
  { kTfLiteMmapRo, kTfLiteInt8, (void*)tensor_data5, (TfLiteIntArray*)&tensor_dimension5, 300, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant5))}, },
  { kTfLiteMmapRo, kTfLiteInt8, (void*)tensor_data6, (TfLiteIntArray*)&tensor_dimension6, 30, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant6))}, },
  { kTfLiteArenaRw, kTfLiteInt8, tensor_arena + 48, (TfLiteIntArray*)&tensor_dimension7, 30, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant7))}, },
  { kTfLiteArenaRw, kTfLiteInt8, tensor_arena + 0, (TfLiteIntArray*)&tensor_dimension8, 10, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant8))}, },
  { kTfLiteArenaRw, kTfLiteInt8, tensor_arena + 16, (TfLiteIntArray*)&tensor_dimension9, 3, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant9))}, },
  { kTfLiteArenaRw, kTfLiteInt8, tensor_arena + 0, (TfLiteIntArray*)&tensor_dimension10, 3, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&quant10))}, },
};const NodeInfo_t nodeData[] = {
  { (TfLiteIntArray*)&inputs0, (TfLiteIntArray*)&outputs0, const_cast<void*>(static_cast<const void*>(&opdata0)), OP_FULLY_CONNECTED, },
  { (TfLiteIntArray*)&inputs1, (TfLiteIntArray*)&outputs1, const_cast<void*>(static_cast<const void*>(&opdata1)), OP_FULLY_CONNECTED, },
  { (TfLiteIntArray*)&inputs2, (TfLiteIntArray*)&outputs2, const_cast<void*>(static_cast<const void*>(&opdata2)), OP_FULLY_CONNECTED, },
  { (TfLiteIntArray*)&inputs3, (TfLiteIntArray*)&outputs3, const_cast<void*>(static_cast<const void*>(&opdata3)), OP_SOFTMAX, },
};
static std::vector<void*> overflow_buffers;
static void * AllocatePersistentBuffer(struct TfLiteContext* ctx,
                                       size_t bytes) {
  void *ptr;
  if (current_location - bytes < tensor_boundary) {
    // OK, this will look super weird, but.... we have CMSIS-NN buffers which
    // we cannot calculate beforehand easily.
    ptr = ei_calloc(bytes, 1);
    if (ptr == NULL) {
      printf("ERR: Failed to allocate persistent buffer of size %d\n", (int)bytes);
      return NULL;
    }
    overflow_buffers.push_back(ptr);
    return ptr;
  }

  current_location -= bytes;

  ptr = current_location;
  memset(ptr, 0, bytes);

  return ptr;
}
typedef struct {
  size_t bytes;
  void *ptr;
} scratch_buffer_t;
static std::vector<scratch_buffer_t> scratch_buffers;

static TfLiteStatus RequestScratchBufferInArena(struct TfLiteContext* ctx, size_t bytes,
                                                int* buffer_idx) {
  scratch_buffer_t b;
  b.bytes = bytes;

  b.ptr = AllocatePersistentBuffer(ctx, b.bytes);
  if (!b.ptr) {
    return kTfLiteError;
  }

  scratch_buffers.push_back(b);

  *buffer_idx = scratch_buffers.size() - 1;

  return kTfLiteOk;
}

static void* GetScratchBuffer(struct TfLiteContext* ctx, int buffer_idx) {
  if (buffer_idx > static_cast<int>(scratch_buffers.size()) - 1) {
    return NULL;
  }
  return scratch_buffers[buffer_idx].ptr;
}

static TfLiteTensor* GetTensor(const struct TfLiteContext* context,
                               int tensor_idx) {
  return &tflTensors[tensor_idx];
}

static TfLiteEvalTensor* GetEvalTensor(const struct TfLiteContext* context,
                                       int tensor_idx) {
  return &tflEvalTensors[tensor_idx];
}

} // namespace

TfLiteStatus trained_model_init( void*(*alloc_fnc)(size_t,size_t) ) {
#ifdef EI_CLASSIFIER_ALLOCATION_HEAP
  tensor_arena = (uint8_t*) alloc_fnc(16, kTensorArenaSize);
  if (!tensor_arena) {
    printf("ERR: failed to allocate tensor arena\n");
    return kTfLiteError;
  }
#else
  memset(tensor_arena, 0, kTensorArenaSize);
#endif
  tensor_boundary = tensor_arena;
  current_location = tensor_arena + kTensorArenaSize;
  ctx.AllocatePersistentBuffer = &AllocatePersistentBuffer;
  ctx.RequestScratchBufferInArena = &RequestScratchBufferInArena;
  ctx.GetScratchBuffer = &GetScratchBuffer;
  ctx.GetTensor = &GetTensor;
  ctx.GetEvalTensor = &GetEvalTensor;
  ctx.tensors = tflTensors;
  ctx.tensors_size = 11;
  for(size_t i = 0; i < 11; ++i) {
    tflTensors[i].type = tensorData[i].type;
    tflEvalTensors[i].type = tensorData[i].type;
    tflTensors[i].is_variable = 0;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
    tflTensors[i].allocation_type = tensorData[i].allocation_type;
#else
    tflTensors[i].allocation_type = (tensor_arena <= tensorData[i].data && tensorData[i].data < tensor_arena + kTensorArenaSize) ? kTfLiteArenaRw : kTfLiteMmapRo;
#endif
    tflTensors[i].bytes = tensorData[i].bytes;
    tflTensors[i].dims = tensorData[i].dims;
    tflEvalTensors[i].dims = tensorData[i].dims;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
    if(tflTensors[i].allocation_type == kTfLiteArenaRw){
      uint8_t* start = (uint8_t*) ((uintptr_t)tensorData[i].data + (uintptr_t) tensor_arena);

     tflTensors[i].data.data =  start;
     tflEvalTensors[i].data.data =  start;
    }
    else{
       tflTensors[i].data.data = tensorData[i].data;
       tflEvalTensors[i].data.data = tensorData[i].data;
    }
#else
    tflTensors[i].data.data = tensorData[i].data;
    tflEvalTensors[i].data.data = tensorData[i].data;
#endif // EI_CLASSIFIER_ALLOCATION_HEAP
    tflTensors[i].quantization = tensorData[i].quantization;
    if (tflTensors[i].quantization.type == kTfLiteAffineQuantization) {
      TfLiteAffineQuantization const* quant = ((TfLiteAffineQuantization const*)(tensorData[i].quantization.params));
      tflTensors[i].params.scale = quant->scale->data[0];
      tflTensors[i].params.zero_point = quant->zero_point->data[0];
    }
    if (tflTensors[i].allocation_type == kTfLiteArenaRw) {
      auto data_end_ptr = (uint8_t*)tflTensors[i].data.data + tensorData[i].bytes;
      if (data_end_ptr > tensor_boundary) {
        tensor_boundary = data_end_ptr;
      }
    }
  }
  if (tensor_boundary > current_location /* end of arena size */) {
    printf("ERR: tensor arena is too small, does not fit model - even without scratch buffers\n");
    return kTfLiteError;
  }
  registrations[OP_FULLY_CONNECTED] = Register_FULLY_CONNECTED();
  registrations[OP_SOFTMAX] = Register_SOFTMAX();

  for(size_t i = 0; i < 4; ++i) {
    tflNodes[i].inputs = nodeData[i].inputs;
    tflNodes[i].outputs = nodeData[i].outputs;
    tflNodes[i].builtin_data = nodeData[i].builtin_data;
    tflNodes[i].custom_initial_data = nullptr;
    tflNodes[i].custom_initial_data_size = 0;
    if (registrations[nodeData[i].used_op_index].init) {
      tflNodes[i].user_data = registrations[nodeData[i].used_op_index].init(&ctx, (const char*)tflNodes[i].builtin_data, 0);
    }
  }
  for(size_t i = 0; i < 4; ++i) {
    if (registrations[nodeData[i].used_op_index].prepare) {
      TfLiteStatus status = registrations[nodeData[i].used_op_index].prepare(&ctx, &tflNodes[i]);
      if (status != kTfLiteOk) {
        return status;
      }
    }
  }
  return kTfLiteOk;
}

static const int inTensorIndices[] = {
  0, 
};
TfLiteTensor* trained_model_input(int index) {
  return &ctx.tensors[inTensorIndices[index]];
}

static const int outTensorIndices[] = {
  10, 
};
TfLiteTensor* trained_model_output(int index) {
  return &ctx.tensors[outTensorIndices[index]];
}

TfLiteStatus trained_model_invoke() {
  for(size_t i = 0; i < 4; ++i) {
    TfLiteStatus status = registrations[nodeData[i].used_op_index].invoke(&ctx, &tflNodes[i]);

#if EI_CLASSIFIER_PRINT_STATE
    ei_printf("layer %lu\n", i);
    ei_printf("    inputs:\n");
    for (size_t ix = 0; ix < tflNodes[i].inputs->size; ix++) {
      auto d = tensorData[tflNodes[i].inputs->data[ix]];

      size_t data_ptr = (size_t)d.data;

      if (d.allocation_type == kTfLiteArenaRw) {
        data_ptr = (size_t)tensor_arena + data_ptr;
      }

      if (d.type == TfLiteType::kTfLiteInt8) {
        int8_t* data = (int8_t*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes; jx++) {
          ei_printf("%d ", data[jx]);
        }
      }
      else {
        float* data = (float*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes / 4; jx++) {
          ei_printf("%f ", data[jx]);
        }
      }
      ei_printf("\n");
    }
    ei_printf("\n");

    ei_printf("    outputs:\n");
    for (size_t ix = 0; ix < tflNodes[i].outputs->size; ix++) {
      auto d = tensorData[tflNodes[i].outputs->data[ix]];

      size_t data_ptr = (size_t)d.data;

      if (d.allocation_type == kTfLiteArenaRw) {
        data_ptr = (size_t)tensor_arena + data_ptr;
      }

      if (d.type == TfLiteType::kTfLiteInt8) {
        int8_t* data = (int8_t*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes; jx++) {
          ei_printf("%d ", data[jx]);
        }
      }
      else {
        float* data = (float*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes / 4; jx++) {
          ei_printf("%f ", data[jx]);
        }
      }
      ei_printf("\n");
    }
    ei_printf("\n");
#endif // EI_CLASSIFIER_PRINT_STATE

    if (status != kTfLiteOk) {
      return status;
    }
  }
  return kTfLiteOk;
}

TfLiteStatus trained_model_reset( void (*free_fnc)(void* ptr) ) {
#ifdef EI_CLASSIFIER_ALLOCATION_HEAP
  free_fnc(tensor_arena);
#endif
  scratch_buffers.clear();
  for (size_t ix = 0; ix < overflow_buffers.size(); ix++) {
    free(overflow_buffers[ix]);
  }
  overflow_buffers.clear();
  return kTfLiteOk;
}