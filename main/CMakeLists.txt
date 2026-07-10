# bPuppy main 组件
# 依赖通过 idf_component.yml 声明，不需要显式 REQUIRES
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "." "../drivers"
)
