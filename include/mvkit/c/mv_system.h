#ifndef MVKIT_C_MV_SYSTEM_H
#define MVKIT_C_MV_SYSTEM_H

/** @file
 * @brief HALCON 风格运行时系统参数。 */

#include <stddef.h>
#include <stdint.h>

#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_tuple.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 设置当前系统参数。
 *
 * @details HALCON C2 控制元组。Set 名称非空，每名称一个值；Get 按顺序拼接查询，接受空查询元组。
 * thread_num 为进程级正 worker 预算或 "default"；builtin 在池启动时冻结预算，serial 只接受
 * 1/default。
 * 空任务/内联工作不启动池。只读查询：thread_pool_size、parallel_backend、cpu_logical_count、
 * cpu_available_count、cpu_physical_count、cpu_performance_core_count、cpu_efficiency_core_count。
 * CPU 数在首次查询时捕获，未知物理/分类数为零；分类数描述系统，不是进程亲和性子集或速度比。
 *
 * @param[in] SystemParameter 参数或数据项名称元组；调用期间只读借用。
 * @param[in] Value 保存、追加或读取的数据元组；调用期间只读借用。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_set_system(MvTuple SystemParameter, MvTuple Value);

/**
 * @brief 查询当前系统参数。
 *
 * @details version 返回当前 MVKit 版本字符串；其他参数按支持的系统配置查询。
 *
 * @param[in] Query 待查询的系统参数名称元组；调用期间只读借用。
 * @param[out] Information 求解或拟合的信息量元组；成功后由调用方使用 mv_destroy_tuple 释放。
 *
 * @return 成功返回 MV_STATUS_OK，失败返回对应的 MvStatus 错误码。
 */
MV_API MvStatus mv_get_system(MvTuple Query, MvTuple* Information);

#ifdef __cplusplus
}
#endif

#endif
