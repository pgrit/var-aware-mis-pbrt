import subprocess
import re
import os
import glob

from testtools import *

# add additional integrator parameters etc here
bdpt_integrator = 'Integrator "bdpt" "integer maxdepth" [1] "bool visualizefactors" "false" '

strategies = { # as expected, the only method coming close to ours is reciprocal variance
    # 'recip-var': '"string misstrategy" "variance" ',
    # 'rel-var': '"string misstrategy" "relativevariance" ',
    'moment': '"string misstrategy" "momentovervar" ',
    # 'rel-dev': '"string misstrategy" "relativedeviation" ',
    # 'rel-var-plus-one': '"string misstrategy" "relvarplusone" ',
    # 'recip-var-plus-one': '"string misstrategy" "recipvarplusone" ',

    # 'recip-var-force': '"string misstrategy" "variance" "bool forceltone" "true" ',
    # 'moment-force': '"string misstrategy" "momentovervar" "bool forceltone" "true" ',
}

modes = {
    # 'injected': '"string weightingmode" "injected" ',
    'reweighted': '"string weightingmode" "reweighted" ',
}

variants = {
    # 'weighted' : '"bool weightedvariance" "true" ',  # worse in all scenes, as expected
    'unweighted' : '"bool weightedvariance" "false" '
}

scenes = {
    'breakfast' : {
        'path': '../../pbrt-v3-scenes/breakfast/',
        'template': 'template.pbrt'
    },
    # 'livingroom' : {
    #     'path': '../../pbrt-v3-scenes/white-room/',
    #     'template': 'template.pbrt'
    # },
    # 'bathroom' : {
    #     'path': '../../pbrt-v3-scenes/bathroom/',
    #     'template': 'template.pbrt'
    # }
}

experiment_sampler = 'Sampler "random" "integer pixelsamples" 8'
reference_sampler = 'Sampler "random" "integer pixelsamples" 128'

def bidir_tester(scene_name, scene_desc):
    filenames = []
    # test all configurations
    for sname, sparams in strategies.items():
        for mname, mparams in modes.items():
            for vname, vparams in variants.items():
                workingDir = './' + scene_name + '/' + sname + '_' + mname #+ '_' + vname
                if not os.path.exists(workingDir):
                    os.makedirs(workingDir)

                # delete images from old runs
                for f in glob.glob(workingDir + '/' + '*.exr'):
                    os.remove(f)

                sc = set_integrator(scene, bdpt_integrator + sparams + mparams + vparams)
                sc = set_sampler(sc, experiment_sampler)
                with open(scene_path + 'scene.pbrtgen', 'w') as f:
                    f.write(sc)
                subprocess.call(['../../pbrt', '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'img.exr'], cwd=workingDir)

                imgs = glob.glob(workingDir + '/' + '*.exr')
                filenames.extend(imgs)
    return filenames

filenames = run_tests('ref-bdpt.exr', bdpt_integrator, reference_sampler, bidir_tester, scenes)
show_results(filenames)