import re

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

        filenames.extend(tester_fn(scene_name, scene_desc))

    return filenames

def show_results(filenames):
    # open all images, assumes tev is in the path
    try:
        viewer = ['tev']
        viewer += filenames
        subprocess.call(viewer)
    except Exception:
        # tev was not found. Maybe we are on WSL and tev is a Windows .exe?
        viewer = ['tev.exe']
        viewer += filenames
        subprocess.call()