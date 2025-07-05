import os
import time

Import("env")


def generate_production_binary(source, target, env):
    signature_bin = "data/signature.bin"
    firmware_hex = "$BUILD_DIR/${PROGNAME}.hex"
    bootloader_hex = "data/bootloader.hex"
    production_image = os.path.join("$BUILD_DIR", "production.hex")
    dfu_package = os.path.join("$BUILD_DIR", "dfu.zip")
    merge_tool = os.path.join(
        env.PioPlatform().get_package_dir("tool-sreccat") or "", "srec_cat"
    )

    # Get actual paths with variable substitution
    firmware_hex_path = env.subst(firmware_hex)
    production_image_path = env.subst(production_image)
    dfu_package_path = env.subst(dfu_package)
    
    # Check if we need to regenerate production image
    need_production = True
    if os.path.isfile(production_image_path) and os.path.isfile(firmware_hex_path):
        # Check if firmware is newer than production image
        firmware_time = os.path.getmtime(firmware_hex_path)
        production_time = os.path.getmtime(production_image_path)
        if firmware_time <= production_time:
            need_production = False
    
    if need_production:
        assert os.path.isfile(bootloader_hex), "Missing bootloader image!"
        assert os.path.isfile(signature_bin), "Missing signature file!"
        env.Execute(
            env.VerboseAction(
                f'"{merge_tool}" "{bootloader_hex}" -intel "{signature_bin}" -Binary -offset 0xFF000 "{firmware_hex}" -intel -offset 0x00 -o "{production_image}" -intel',
                f"Generating production image {production_image}",
            )
        )
    
    # Check if we need to regenerate DFU package
    need_dfu = True
    if os.path.isfile(dfu_package_path) and os.path.isfile(production_image_path):
        # Check if production image is newer than DFU package
        production_time = os.path.getmtime(production_image_path)
        dfu_time = os.path.getmtime(dfu_package_path)
        if production_time <= dfu_time:
            need_dfu = False
    
    if need_dfu:
        env.Execute(
            env.VerboseAction(
                f'adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application "{firmware_hex}" "{dfu_package}"',
                f"Generating DFU package {dfu_package}",
            )
        )

# Run this function after every build
env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", generate_production_binary)