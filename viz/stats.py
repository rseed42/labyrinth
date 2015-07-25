class FrameStats(object):
    def __init__(self, fps):
        self.fps = fps
        self.target = 1000./fps
        self.count = 0
        self.inputSum = 0
        self.simulationSum = 0
        self.renderSum = 0
        self.agentSum = 0
        self.totalSum = 0
        self.delaySum = 0

    def show(self):
        print '='*40
        print 'Frame Statistics'
        print '='*40
        print 'Targets'
        print '-'*40
        print 'FPS: %.1f' % self.fps
        print 'Frame: %.2f ms' % self.target
        print '-'*40
        print 'Measured'
        print '-'*40
        print 'Frames: %d' % self.count
        fcount = float(self.count)
        print 'Mean input: %.4f ms' % (self.inputSum / fcount)
        print 'Mean simulation: %.4f ms' % (self.simulationSum / fcount)
        print 'Mean render: %.4f ms' % (self.renderSum / fcount)
        print 'Mean agent: %.4f ms' % (self.agentSum / fcount)
        print 'Mean total: %.4f ms' % (self.totalSum / fcount)
        print 'Mean delay: %.4f ms' % (self.delaySum / fcount)
