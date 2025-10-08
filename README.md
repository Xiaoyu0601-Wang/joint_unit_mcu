# joint_unit_mcu_nuttx

## Install dependency
```sh
sudo apt install python3-pip
sudo apt install python3-yaml
```

## Download Nuttx dependency
```sh
python3 download_nuttx_repos.py
# Deprecated
# vcs import joint_unit_mcu_nuttx < nuttx.repos
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