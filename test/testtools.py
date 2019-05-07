import re
import os
import glob
import subprocess
from subprocess import Popen, PIPE
import numpy as np

# generates the string with the selected integrator
def set_integrator(scene, integrator_str):
    start = '##INTEGRATOR-DEF-START'
    end = '##INTEGRATOR-DEF-END'
    replacement = integrator_str
    match = re.match(r'(.+%s\s*).+?(\s*%s.+)' % (start, end), scene, re.DOTALL)
    return match.group(1) + replacement + match.group(2)

def set_sampler(scene, sampler_str):
    start = '##SAMPLER-DEF-START'
    end = '##SAMPLER-DEF-END'
    replacement = sampler_str
    match = re.match(r'(.+%s\s*).+?(\s*%s.+)' % (start, end), scene, re.DOTALL)
    return match.group(1) + replacement + match.group(2)

def run_and_time(args, workingDir, repeats=1):
    totalTime = 0.0

    var = 0.0
    mean = 0.0
    n = 0

    for k in range(repeats):
        p = Popen(args, cwd=workingDir, stdin=PIPE, stdout=PIPE, stderr=PIPE)
        output, err = p.communicate()

        # Format of our implementation
        renderTime = re.findall(r'Total rendering time: (\d+\.\d+) seconds.', output.decode('utf-8'))
        overheadTime = re.findall(r'Overhead: (\d+\.\d+) seconds.', output.decode('utf-8'))

        trialTime = 0.0

        if not renderTime:
            # Format of the optimal MIS implementation
            renderTime = re.findall(r'Rendering stats: samples \d+, time (\d+\.\d+) s', output.decode('utf-8'))

            if not renderTime:
                # Fallback: Hijack PBRT progress reporter
                # Accuracy below 0.5 seconds!!
                # Might also be printing the full time multiple times,
                # the first output is right after Done() was called and should be most accurate
                times = re.findall(r'\+\+\]  \((\d+\.\d+)s\)', output.decode('utf-8'))
                times = np.array(times, dtype=np.float32)
                trialTime = times[0]
                print("Warning: time measurement fallback option triggered, accuracy < 0.5s!")
            else:
                trialTime = float(renderTime[0])
        else:
            trialTime = float(renderTime[0])

        n += 1
        if n == 1:
            mean = trialTime
        else:
            newMean = mean + (trialTime - mean) / n
            var += (trialTime - mean) * (trialTime - newMean)
            mean = newMean

    if n > 1:
        var /= n-1
    twoStandardDevs = np.sqrt(var) * 2

    import math
    roundToN = lambda x, n: round(x, -int(math.floor(math.log10(x))) + (n-1))
    return (roundToN(mean, 3), 0.0 if repeats == 1 else roundToN(twoStandardDevs, 3))

def run_tests(ref_name, ref_integrator, ref_sampler, tester_fn, scenes):
    filenames = []
    for scene_name, scene_desc in scenes.items():
        scene_path = scene_desc['path']
        if not os.path.exists('./' + scene_name):
            os.makedirs('./' + scene_name)

        # load the scene template and render the reference (if it does not exist already)
        with open(scene_path + scene_desc['template'], 'r') as f:
            scene = f.read()

        refpath = scene_name + '/' + ref_name
        if not os.path.isfile(refpath):
            sc = set_integrator(scene, ref_integrator)
            sc = set_sampler(sc, ref_sampler)
            with open(scene_path + 'scene.pbrtgen', 'w') as f:
                f.write(sc)
            subprocess.call(['./pbrt', scene_path + 'scene.pbrtgen', '--outfile', refpath])

        filenames.append(refpath)

        filenames.extend(tester_fn(scene_name, scene, scene_path))

    return filenames

def show_results(filenames):
    # separate out the stratification factors
    factorImages = []
    for name in filenames:
        if 'stratfactor-d' in name:
            factorImages.append(name)
    for name in factorImages:
        filenames.remove(name)

    # open all images, assumes tev is in the path
    try:
        viewer = ['tev']
        viewer += filenames
        subprocess.call(viewer)
    except Exception:
        # tev was not found. Maybe we are on WSL and tev is a Windows .exe?
        viewer = ['tev.exe']
        viewer += filenames
        try:
            subprocess.call(viewer)
        except:
            print('"tev" not found in path, proceeding without showing images')