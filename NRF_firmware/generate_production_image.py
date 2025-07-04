import os

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

    if not os.path.isfile(env.subst(production_image)):
        assert os.path.isfile(bootloader_hex), "Missing bootloader image!"
        assert os.path.isfile(signature_bin), "Missing signature file!"
        env.Execute(
            env.VerboseAction(
                f'"{merge_tool}" "{bootloader_hex}" -intel "{signature_bin}" -Binary -offset 0xFF000 "{firmware_hex}" -intel -offset 0x00 -o "{production_image}" -intel',
                f"Generating production image {production_image}",
            )
        )
    
    # Generate DFU package
    if not os.path.isfile(env.subst(dfu_package)):
        env.Execute(
            env.VerboseAction(
                f'adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application "{firmware_hex}" "{dfu_package}"',
                f"Generating DFU package {dfu_package}",
            )
        )

env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", generate_production_binary)