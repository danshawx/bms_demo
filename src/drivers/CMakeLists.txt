

# Application sources
# add_subdirectory(nw_cat1)

# add_subdirectory_ifdef(CONFIG_MODEM_CAT1 nw_cat1)
add_subdirectory_ifdef(CONFIG_BQ769x2 bq769x2)
add_subdirectory_ifdef(CONFIG_LIS2DW12XD lis2dw12)
