#!/usr/bin/env python3
"""Compile the autonomous against real dependencies and run it with simulated navigation.

Normal use after Gradle sync: python3 TeamCode/tests/run_autonomous.py
Offline source audit: add --upstream-source ROOT containing pedro/ and ivy/ sources
from the unmodified Pedro v3.0.2 and Ivy v1.1.1 tags. This is not an APK build.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--upstream-source', type=Path)
args = parser.parse_args()
subprocess.run([shutil.which('python3') or 'python3', str(ROOT / 'TeamCode/tools/sync_autonomous_route.py'), '--check'], check=True)
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


with tempfile.TemporaryDirectory(prefix='biobuzz-auto-') as temporary:
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
    sources = list((ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous').glob('*.java'))
    sources += [ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java',
                ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/AutoPath.java',
                ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java',
                ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzAuto.java',
                Path(__file__).with_name('BioBuzzAutoRegression.java'),
                Path(__file__).with_name('AutoPathRegression.java'),
                Path(__file__).with_name('MechanismIsolationRegression.java'),
                Path(__file__).with_name('AutoLifecycleRegression.java')]
    sources += [p for p in (ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode').glob('*.java')
                if p not in sources and p.name != 'Tuning.java']
    sources += [Path(__file__).with_name('MechanismTestRegression.java')]
    if args.upstream_source:
        sources += sorted(args.upstream_source.rglob('*.java'))
        print('Compiling against supplied upstream sources (not resolved Maven binaries).', flush=True)
    else:
        dependency('com.pedropathing', 'core', '3.0.2', 'jar')
        dependency('com.pedropathing', 'revhub', '3.0.2', 'aar')
        dependency('com.pedropathing.ivy', 'core', '1.1.1', 'jar')
        dependency('com.pedropathing.ivy', 'pedro', '1.1.1', 'aar')
    classpath = os.pathsep.join(map(str, jars))
    subprocess.run([java_tool('javac'), '-source', '8', '-target', '8', '-proc:none',
                    '-cp', classpath, '-d', str(work)] + list(map(str, sources)), check=True)
    subprocess.run([java_tool('java'), '-cp', os.pathsep.join([str(work), classpath]),
                    'org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoRegression'], check=True)

    subprocess.run([java_tool('java'), '-cp', os.pathsep.join([str(work), classpath]),
                    'org.firstinspires.ftc.teamcode.AutoPathRegression'], check=True)

    subprocess.run([java_tool('java'), '-cp', os.pathsep.join([str(work), classpath]),
                    'org.firstinspires.ftc.teamcode.autonomous.MechanismIsolationRegression'], check=True)

    subprocess.run([java_tool('java'), '-cp', os.pathsep.join([str(work), classpath]),
                    'org.firstinspires.ftc.teamcode.autonomous.AutoLifecycleRegression'], check=True)

    subprocess.run([java_tool('java'), '-cp', os.pathsep.join([str(work), classpath]),
                    'org.firstinspires.ftc.teamcode.autonomous.MechanismTestRegression'], check=True)
