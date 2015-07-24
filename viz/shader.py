class Shader(object):
    def __init__(self):
        self.src = ""

    def load(self, shader_dir, cfg):
        fp = file(shader_dir+cfg.filename, 'r')
        self.src = fp.read()
        fp.close()
