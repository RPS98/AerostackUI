class Marker extends DrawManager
{
    constructor(name, codeDrawOptions={}, userDrawOptions={}) {
        super('Marker', name, codeDrawOptions, userDrawOptions);
    }

    codeDraw(layer, values, options={}) {
        return super.codeDraw(layer, values, options={});
    }

    userDraw(layer, options={}) {
        return super.userDraw(layer, options={});
    }

    showInfo() {
        throw new Error("Method not implemented.");
    }

    sendInfo() {
        throw new Error("Method not implemented.");
    }
}

class PointOfInterest extends Marker
{
    constructor(codeDrawOptions={}, userDrawOptions={}) {
        super('PointOfInterest', codeDrawOptions, userDrawOptions);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}


class WayPoint extends Marker
{
    constructor(codeDrawOptions={}, userDrawOptions={}) {
        super('WayPoint', codeDrawOptions, userDrawOptions);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}

class LandPoint extends Marker
{
    constructor(codeDrawOptions={}, userDrawOptions={}) {
        super('LandPoint', codeDrawOptions, userDrawOptions);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}


class UAVMarker extends Marker
{
    constructor(codeDrawOptions={}, userDrawOptions={}) {
        super('UAVMarker', codeDrawOptions, userDrawOptions);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}