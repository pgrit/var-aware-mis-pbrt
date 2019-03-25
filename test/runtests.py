import subprocess
import re
import os
import glob

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


# add additional integrator parameters etc here
integrators = {
    # 'path': 'Integrator "path"', # TODO add support for the path tracer (with all the same parameters etc)
    'bdpt': 'Integrator "bdpt" "integer maxdepth" [1] "bool visualizefactors" "false" '
}

strategies = { # as expected, the only method coming close to ours is reciprocal variance
    'recip-var': '"string misstrategy" "variance" ',
    # 'rel-var': '"string misstrategy" "relativevariance" ',
    'moment': '"string misstrategy" "momentovervar" ',
    # 'rel-dev': '"string misstrategy" "relativedeviation" ',
    # 'rel-var-plus-one': '"string misstrategy" "relvarplusone" ',
    # 'recip-var-plus-one': '"string misstrategy" "recipvarplusone" ',
}

modes = {
    'injected': '"string weightingmode" "injected" ',
}

variants = {
    # 'weighted' : '"bool weightedvariance" "true" ',  # worse in all scenes, as expected
    'unweighted' : '"bool weightedvariance" "false" '
}

experiment_sampler = 'Sampler "random" "integer pixelsamples" 8'
reference_sampler = 'Sampler "random" "integer pixelsamples" 128'

scenes = {
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

filenames = []

for scene_name, scene_desc in scenes.items():
    scene_path = scene_desc['path']
    if not os.path.exists('./' + scene_name):
        os.makedirs('./' + scene_name)

    # load the scene template and render the reference (if it does not exist already)
    with open(scene_path + scene_desc['template'], 'r') as f:
        scene = f.read()

    if not os.path.isfile('./' + scene_name + '/ref.exr'):
        sc = set_integrator(scene, integrators['bdpt'])
        sc = set_sampler(sc, reference_sampler)
        with open(scene_path + 'scene.pbrt', 'w') as f:
            f.write(sc)
        subprocess.call(['./pbrt', scene_path + 'scene.pbrt', '--outfile', scene_name + '/ref.exr'])

    filenames.append('./' + scene_name + '/ref.exr')

    # test all configurations
    for name, integrator in integrators.items():
        for sname, sparams in strategies.items():
            for mname, mparams in modes.items():
                for vname, vparams in variants.items():
                    workingDir = './' + scene_name + '/' + sname + '_' + mname #+ '_' + vname
                    if not os.path.exists(workingDir):
                        os.makedirs(workingDir)

                    # delete images from old runs
                    for f in glob.glob(workingDir + '/' + '*.exr'):
                        os.remove(f)

                    sc = set_integrator(scene, integrator + sparams + mparams + vparams)
                    sc = set_sampler(sc, experiment_sampler)
                    with open(scene_path + 'scene.pbrt', 'w') as f:
                        f.write(sc)
                    subprocess.call(['../../pbrt', '../../' + scene_path + 'scene.pbrt', '--outfile', 'img.exr'], cwd=workingDir)

                    imgs = glob.glob(workingDir + '/' + '*.exr')
                    filenames.extend(imgs)

# open all images, assumes tev is in the path (and we are on Windows / WSL)
# TODO find portable way to call tev here (cannot remove .exe because of WSL)
#      maybe catch error and call plain 'tev' instead
viewer = ['tev.exe']
viewer += filenames
# print(viewer)
subprocess.call(viewer)