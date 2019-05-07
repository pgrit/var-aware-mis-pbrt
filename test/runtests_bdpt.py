import subprocess
import re
import os
import glob

from testtools import *

benchmarkRepeats = 1

# add additional integrator parameters etc here
bdpt_integrator = 'Integrator "bdpt" "integer maxdepth" [5] ' + ' "float clampthreshold" 2.0 ' + ' "integer rectimaxdepth" [5] ' + ' "integer downsamplingfactor" 8 '
bdpt_integrator += ' "bool visualizefactors" "true" '

bdpt_integrator_di = 'Integrator "bdpt" "integer maxdepth" [1] ' + ' "float clampthreshold" 2.0 ' + ' "integer rectimaxdepth" [1] ' + ' "integer downsamplingfactor" 8 '
bdpt_integrator_di += ' "bool visualizefactors" "true" '

pt_integrator = 'Integrator "path" "integer maxdepth" [5] "string lightsamplestrategy" "power" '
pt_integrator_di = 'Integrator "path" "integer maxdepth" [1] "string lightsamplestrategy" "power" '

reference_integrator = 'Integrator "bdpt" "integer maxdepth" [5] "bool visualizefactors" "false" ' + ' "float clampthreshold" 2.0 '

experiment_sampler = 'Sampler "random" "integer pixelsamples" 8'
double_sampler = 'Sampler "random" "integer pixelsamples" 16'
reference_sampler = 'Sampler "random" "integer pixelsamples" 1024'

moment = ' "string mismod" "moment" '
variance = ' "string mismod" "reciprocal" '
vanilla = ' "string mismod" "none" '
balance = ' "string misstrategy" "balance" '
power = ' "string misstrategy" "power" '
nomis = ' "string misstrategy" "uniform" '

# different integrator configurations to test
variants = {
    'bdpt-balance': balance + vanilla,
    'bdpt-power': power + vanilla,
    'bdpt-nomis': nomis + vanilla,
    'bdpt-our-balance': balance + moment,
    'bdpt-recipvar-only': nomis + variance,
    'bdpt-our-power': power + moment,
}

scenes = {
    'veach-mis' : {
        'path': '../../pbrt-v3-scenes/veach-mis/',
        'template': 'template.pbrt'
    },
    'breakfast' : {
        'path': '../../pbrt-v3-scenes/breakfast/',
        'template': 'template.pbrt'
    },
    'livingroom' : {
        'path': '../../pbrt-v3-scenes/white-room/',
        'template': 'template.pbrt'
    },
    'bathroom' : {
        'path': '../../pbrt-v3-scenes/bathroom/',
        'template': 'template.pbrt'
    }
}

def bidir_tester(scene_name, scene, scene_path):
    print('Testing: ' + scene_name)
    print('==============================')
    filenames = []
    # test all configurations
    for vname, vparams in variants.items():
        workingDir = './' + scene_name + '/' + vname
        if not os.path.exists(workingDir):
            os.makedirs(workingDir)

        # delete images from old runs
        for f in glob.glob(workingDir + '/' + '*.exr'):
            os.remove(f)

        sc = set_integrator(scene, bdpt_integrator + vparams)
        sc = set_sampler(sc, experiment_sampler)
        with open(scene_path + 'scene.pbrtgen', 'w') as f:
            f.write(sc)
        time = run_and_time(['../../pbrt', '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'img.exr'], workingDir, repeats=benchmarkRepeats)
        print(vname + ' took ' + str(time[0]) + ' s (+- ' + str(time[1]) + ' s)')

        sc = set_integrator(scene, bdpt_integrator_di + vparams)
        sc = set_sampler(sc, experiment_sampler)
        with open(scene_path + 'scene.pbrtgen', 'w') as f:
            f.write(sc)
        time = run_and_time(['../../pbrt', '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'direct-only.exr'], workingDir, repeats=benchmarkRepeats)
        print(vname + ' (direct illum. only) took ' + str(time[0]) + ' s (+- ' + str(time[1]) + ' s)')

        imgs = glob.glob(workingDir + '/' + '*.exr')
        filenames.extend(imgs)

    # path tracer equal sample and equal iteration baseline comparisons
    workingDir = './' + scene_name + '/path'
    if not os.path.exists(workingDir):
        os.makedirs(workingDir)

    # delete images from old runs
    for f in glob.glob(workingDir + '/' + '*.exr'):
        os.remove(f)

    sc = set_integrator(scene, pt_integrator)
    sc = set_sampler(sc, double_sampler)
    with open(scene_path + 'scene.pbrtgen', 'w') as f:
        f.write(sc)
    time = run_and_time(['../../pbrt', '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'path-double.exr'], workingDir, repeats=benchmarkRepeats)
    print('path tracer (double) took ' + str(time[0]) + ' s (+- ' + str(time[1]) + ' s)')

    sc = set_integrator(scene, pt_integrator_di)
    sc = set_sampler(sc, double_sampler)
    with open(scene_path + 'scene.pbrtgen', 'w') as f:
        f.write(sc)
    time = run_and_time(['../../pbrt', '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'path-double-direct-only.exr'], workingDir, repeats=benchmarkRepeats)
    print('path tracer (double, direct illum. only) took ' + str(time[0]) + ' s (+- ' + str(time[1]) + ' s)')

    sc = set_integrator(scene, pt_integrator)
    sc = set_sampler(sc, experiment_sampler)
    with open(scene_path + 'scene.pbrtgen', 'w') as f:
        f.write(sc)
    time = run_and_time(['../../pbrt', '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'path-same.exr'], workingDir, repeats=benchmarkRepeats)
    print('path tracer (same) took ' + str(time[0]) + ' s (+- ' + str(time[1]) + ' s)')

    sc = set_integrator(scene, pt_integrator_di)
    sc = set_sampler(sc, experiment_sampler)
    with open(scene_path + 'scene.pbrtgen', 'w') as f:
        f.write(sc)
    time = run_and_time(['../../pbrt', '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'path-same-direct-only.exr'], workingDir, repeats=benchmarkRepeats)
    print('path tracer (same, direct illum. only) took ' + str(time[0]) + ' s (+- ' + str(time[1]) + ' s)')

    imgs = glob.glob(workingDir + '/' + '*.exr')
    filenames.extend(imgs)

    print('==============================')

    return filenames

filenames = run_tests('ref-bdpt.exr', reference_integrator, reference_sampler, bidir_tester, scenes)

# use this to open all results right away
# show_results(filenames)