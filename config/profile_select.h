#ifndef PROFILE_SELECT_H
#define PROFILE_SELECT_H

/* 产品Profile只在编译时选择；不同Profile可能占用不同UART和任务集合。 */
#define CAR_PROFILE_FULL               (0U)
#define CAR_PROFILE_GMR                (1U)

/*
 * 新构建使用CAR_ACTIVE_PROFILE。保留旧宏输入兼容，便于现有CCS配置过渡；
 * 实现层只读取CAR_PROFILE_IS_*，不要再直接新增旧宏分支。
 */
#ifndef CAR_ACTIVE_PROFILE
#if defined(CAR_LIBRARY_GMR_CONFIG_ENABLED)
#if CAR_LIBRARY_GMR_CONFIG_ENABLED
#define CAR_ACTIVE_PROFILE             CAR_PROFILE_GMR
#else
#define CAR_ACTIVE_PROFILE             CAR_PROFILE_FULL
#endif
#else
#define CAR_ACTIVE_PROFILE             CAR_PROFILE_GMR
#endif
#endif

#if ((CAR_ACTIVE_PROFILE != CAR_PROFILE_FULL) && \
    (CAR_ACTIVE_PROFILE != CAR_PROFILE_GMR))
#error "CAR_ACTIVE_PROFILE must be CAR_PROFILE_FULL or CAR_PROFILE_GMR"
#endif

#define CAR_PROFILE_IS_FULL \
    (CAR_ACTIVE_PROFILE == CAR_PROFILE_FULL)
#define CAR_PROFILE_IS_GMR \
    (CAR_ACTIVE_PROFILE == CAR_PROFILE_GMR)

/* 旧名称作为只读兼容别名，现有模块可逐步迁移。 */
#ifdef CAR_LIBRARY_GMR_CONFIG_ENABLED
#undef CAR_LIBRARY_GMR_CONFIG_ENABLED
#endif
#define CAR_LIBRARY_GMR_CONFIG_ENABLED CAR_PROFILE_IS_GMR

#if CAR_PROFILE_IS_GMR
#define CAR_ACTIVE_PROFILE_NAME        "Gmr"
#else
#define CAR_ACTIVE_PROFILE_NAME        "Full"
#endif

#endif
