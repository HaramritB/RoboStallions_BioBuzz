#!/usr/bin/env python3
"""Run hardware-free Robot regression checks using the project's cached FTC SDK."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[2]
gradle_cache = Path(os.environ.get("GRADLE_USER_HOME", Path.home() / ".gradle")) / "caches/modules-2/files-2.1"
sdk = os.environ.get("ANDROID_HOME") or os.environ.get("ANDROID_SDK_ROOT")
if not sdk:
    for line in (ROOT / "local.properties").read_text().splitlines():
        if line.startswith("sdk.dir="):
            sdk = line.partition("=")[2]
if not sdk:
    raise SystemExit("Set ANDROID_HOME or sdk.dir in local.properties.")
android_jars = sorted((Path(sdk) / "platforms").glob("android-*/android.jar"),
                      key=lambda p: int(p.parent.name.split("-")[1]))
if not android_jars:
    raise SystemExit("No installed Android SDK platform found.")
java_home = os.environ.get("JAVA_HOME")
studio_java = Path("/Applications/Android Studio.app/Contents/jbr/Contents/Home")
if not java_home and studio_java.exists():
    java_home = str(studio_java)


def java_tool(name):
    return str(Path(java_home) / "bin" / name) if java_home else shutil.which(name) or name


with tempfile.TemporaryDirectory(prefix="biobuzz-controls-") as temp:
    work = Path(temp)
    jars = [str(android_jars[-1])]
    for module in ("RobotCore", "Hardware"):
        aars = list((gradle_cache / "org.firstinspires.ftc" / module / "12.0.0").glob("*/*.aar"))
        if not aars:
            raise SystemExit(f"FTC {module} 12.0.0 not cached; sync the project first.")
        destination = work / f"{module}.jar"
        with zipfile.ZipFile(aars[0]) as archive:
            destination.write_bytes(archive.read("classes.jar"))
        jars.append(str(destination))
    classpath = os.pathsep.join(jars)
    subprocess.run([java_tool("javac"), "-proc:none", "-cp", classpath, "-d", str(work),
                    str(ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java"),
                    str(Path(__file__).with_name("RobotControlsRegression.java"))], check=True)
    subprocess.run([java_tool("java"), "-cp", os.pathsep.join([str(work), classpath]),
                    "org.firstinspires.ftc.teamcode.RobotControlsRegression"], check=True)
