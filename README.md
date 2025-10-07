# raspberry_pico_nuttx

## Download dependency

```sh
vcs import . < nuttx.repos
```

# Joint Unit MCU Nuttx
```sh
nuttx/
apps/
asr_sdm_drivers/
  ├── Kconfig
  ├── Makefile
  └── icm42688/
      ├── Kconfig
      ├── Makefile
      └── icm42688.c
  └── dynamixel/
      ├── Kconfig
      ├── Makefile
      └── dynamixel.c
asr_sdm_apps/
  ├── Kconfig
  ├── Makefile
  └── icm42688/
      ├── Kconfig
      ├── Makefile
      └── icm42688.c
```

## Reference link
https://nuttx.apache.org/docs/latest/guides/building_nuttx_with_app_out_of_src_tree.html#make-export