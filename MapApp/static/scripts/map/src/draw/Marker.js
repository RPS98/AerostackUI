class Marker extends DrawManager
{
    constructor(name, codeDrawOptions={}, userDrawOptions={}) {
        super('Marker', name, codeDrawOptions, userDrawOptions);
    }

    codeDraw(values, options={}) {
        return super.codeDraw(values, options);
    }

    userDraw(options={}) {
        super.userDraw(options);
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

    codeDraw(id, values, options={}) {
        options['icon'] = new L.Icon({
            iconUrl: M.UAV_MANAGER.getIcon(id),
            iconSize: [30, 30],
            iconAnchor: [15, 15],
            popupAnchor: [0, -15]
        });
        return super.codeDraw(values, options);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}