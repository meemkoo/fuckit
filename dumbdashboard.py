import ntcore

class DumbDashboard:
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("DumbDashboard")
    things: dict[str, dict[str, ntcore.StringEntry]] = {}

    @classmethod
    def grab(cls, name, default, type_=str):
        default = str(default)
        if name not in cls.things:
            cls.things[name] = {"type": type_, "sub": cls.table.getStringTopic(name).subscribe(default)}
        elif name in cls.things and len(cls.things[name]) == 2:
            print("Reached fine")
            cls.things[name].update({"sub": cls.table.getStringTopic(name).subscribe(default)})
        return cls.things[name]["type"](cls.things[name]["sub"].get(default))

    @classmethod
    def put(cls, name, value, type_=str):
        value = str(value)
        if name not in cls.things:
            cls.things[name] = {"type": type_, "pub": cls.table.getStringTopic(name).publish()}
        elif name in cls.things and len(cls.things[name]) == 2:
            cls.things[name].update({"pub": cls.table.getStringTopic(name).publish()})
        cls.things[name]["pub"].set(value)