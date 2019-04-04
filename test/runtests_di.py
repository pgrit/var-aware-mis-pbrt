from testtools import *

experiment_sampler = 'Sampler "random" "integer pixelsamples" 8'
reference_sampler = 'Sampler "random" "integer pixelsamples" 128'

di_integrator = 'integrator "guideddi" '

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

moment = ' "string varmode" "moment" '
variance = ' "string varmode" "variance" '
vanilla = ' "string varmode" "disabled" '
balance = ' "string mis" "balance" '
power = ' "string mis" "power" '
nomis = ' "string mis" "uniform" '
nobsdf = ' "bool enablebsdf" "false" '
noguided = ' "bool enableguided" "false" '
nouniform = ' "bool enableuniform" "false" '

# different integrator configurations to test
variants = {
    'nobsdf-balance': balance + nobsdf,
    'nobsdf-power': power + nobsdf,
    'nobsdf-nomis': nomis + nobsdf,
    'nobsdf-recipvar': balance + variance + nobsdf,
    'nobsdf-moment': balance + moment + nobsdf,
    'nobsdf-recipvar-only': nomis + variance + nobsdf,
    'nobsdf-moment-only': nomis + moment + nobsdf,
    'nobsdf-recipvar-power': power + variance + nobsdf,
    'nobsdf-moment-power': power + moment + nobsdf,
}

def di_tester(scene_name, scene_config):
    filenames = []
    for vname, vparams in variants.items():
        workingDir = './' + scene_name + '/' + vname
        if not os.path.exists(workingDir):
            os.makedirs(workingDir)

        # delete images from old runs
        for f in glob.glob(workingDir + '/' + '*.exr'):
            os.remove(f)

        sc = set_integrator(scene, di_integrator + vparams)
        sc = set_sampler(sc, experiment_sampler)
        with open(scene_path + 'scene.pbrtgen', 'w') as f:
            f.write(sc)
        subprocess.call(['../../pbrt', '../../' + scene_path + 'scene.pbrtgen', '--outfile', 'img.exr'], cwd=workingDir)

        imgs = glob.glob(workingDir + '/' + '*.exr')
        filenames.extend(imgs)
    return filenames

filenames = run_tests('ref-di.exr', di_integrator, reference_sampler, di_tester, scenes)
show_results(filenames)