class StaticObject(object):
    def __init__(self):
        self.id = None
        self.body = None

    def configure(self, cfg):
        self.id = cfg.id

    def setBody(self, body):
        self.body = body
