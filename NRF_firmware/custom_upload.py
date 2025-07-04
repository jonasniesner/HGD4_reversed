import os

Import("env")

custom_hex_path = env.subst(os.path.join("$BUILD_DIR", "production.hex"))

env.Replace(
    UPLOADER="JLinkExe",
    UPLOADCMD=f"JLinkExe -device nrf52840_xxaa -if swd -speed 4000 -autoconnect 1 -CommanderScript upload.jlink"
)

# Auto-generate the JLink command file from script if needed:
with open("upload.jlink", "w") as f:
    f.write(f"""r
loadfile {custom_hex_path}
r
g
exit
""")
