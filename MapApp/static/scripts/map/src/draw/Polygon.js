class Polygon extends DrawManager
{
    constructor(name, options = undefined) {
        super('Polygon', name, options);
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

class Area extends Polygon
{
    constructor(options = undefined) {
        super('Area', options);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}

