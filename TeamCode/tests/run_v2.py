#!/usr/bin/env python3
"""Compile all TeamCode (including the V2 package) and run the hardware-free V2 checks.

Usage after a Gradle sync: python3 TeamCode/tests/run_v2.py
Uses the locally cached FTC SDK 12.0.0, Pedro 3.0.2 and Ivy 1.1.1, like run_autonomous.py.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[2]
TEAMCODE = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
cache = Path(os.environ.get('GRADLE_USER_HOME', Path.home() / '.gradle')) / 'caches/modules-2/files-2.1'
sdk = os.environ.get('ANDROID_HOME') or os.environ.get('ANDROID_SDK_ROOT')
if not sdk and (ROOT / 'local.properties').exists():
    sdk = next((line.partition('=')[2] for line in (ROOT / 'local.properties').read_text().splitlines()
                if line.startswith('sdk.dir=')), None)
if not sdk:
    raise SystemExit('Set ANDROID_HOME or sdk.dir in local.properties.')
platforms = sorted((Path(sdk) / 'platforms').glob('android-*/android.jar'),
                   key=lambda p: int(p.parent.name.split('-')[1]))
if not platforms:
    raise SystemExit('No Android SDK platform installed.')
java_home = os.environ.get('JAVA_HOME')
studio_java = Path('/Applications/Android Studio.app/Contents/jbr/Contents/Home')
if not java_home and studio_java.exists():
    java_home = str(studio_java)


def java_tool(name):
    return str(Path(java_home) / 'bin' / name) if java_home else shutil.which(name) or name


with tempfile.TemporaryDirectory(prefix='biobuzz-v2-') as temporary:
    work = Path(temporary)
    jars = [platforms[-1]]

    def dependency(group, module, version, extension):
        matches = sorted((cache / group / module / version).glob(f'*/*.{extension}'))
        if not matches:
            raise SystemExit(f'{group}:{module}:{version} not cached; sync Gradle first.')
        path = matches[0]
        if extension == 'aar':
            destination = work / f'{group}-{module}.jar'
            with zipfile.ZipFile(path) as archive:
                destination.write_bytes(archive.read('classes.jar'))
            path = destination
        jars.append(path)

    for module in ('RobotCore', 'Hardware'):
        dependency('org.firstinspires.ftc', module, '12.0.0', 'aar')
    dependency('com.pedropathing', 'core', '3.0.2', 'jar')
    dependency('com.pedropathing', 'revhub', '3.0.2', 'aar')
    dependency('com.pedropathing.ivy', 'core', '1.1.1', 'jar')
    dependency('com.pedropathing.ivy', 'pedro', '1.1.1', 'aar')

    sources = [p for p in TEAMCODE.glob('*.java') if p.name != 'Tuning.java']
    sources += sorted((TEAMCODE / 'autonomous').glob('*.java'))
    sources += sorted((TEAMCODE / 'v2').glob('*.java'))
    sources += [Path(__file__).with_name('V2Regression.java')]
    classpath = os.pathsep.join(map(str, jars))
    subprocess.run([java_tool('javac'), '-source', '8', '-target', '8', '-proc:none', '-nowarn',
                    '-cp', classpath, '-d', str(work)] + list(map(str, sources)), check=True)
    subprocess.run([java_tool('java'), '-cp', os.pathsep.join([str(work), classpath]),
                    'org.firstinspires.ftc.teamcode.v2.V2Regression'], check=True)
