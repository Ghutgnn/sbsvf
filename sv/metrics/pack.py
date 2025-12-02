class CollisionMetric:
    id = "collision"; version = "1.0.0"
    def reset(self, ctx): self.hit = 0
    def on_tick(self, t, bus, events):
        self.hit |= 0  # 教學：先固定 0
    def finalize(self): return {"collision": int(self.hit)}

class RouteCompletion:
    id = "route_completion"; version = "1.0.0"
    def reset(self, ctx): self.progress = 0.98
    def on_tick(self, t, bus, events): pass
    def finalize(self): return {"route_completion": self.progress}

class MetricsPack:
    def __init__(self, metrics): self.metrics = metrics
    @classmethod
    def load(cls, base_dir):
        return cls([CollisionMetric(), RouteCompletion()])
    def reset(self, ctx):
        for m in self.metrics:
            m.reset(ctx)
    def on_tick(self, t, bus, events):
        for m in self.metrics:
            if hasattr(m, 'on_tick'): m.on_tick(t, bus, events)
    def finalize(self):
        out = {}
        for m in self.metrics: out.update(m.finalize())
        return out