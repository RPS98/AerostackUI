class CircleMarker extends DrawManager
{
    constructor(name, options = undefined) {
        super('CircleMarker', name, options);
    }

    codeDraw(values, options={}) {
        return super.codeDraw(values, options);
    }

    userDraw(options={}) {
        return super.userDraw(options);
    }

    showInfo() {
        throw new Error("Method not implemented.");
    }

    sendInfo() {
        throw new Error("Method not implemented.");
    }
}
