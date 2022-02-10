class Polygon extends DrawManager
{
    constructor(name, codeDrawOptions={}, userDrawOptions={}) {
        super('Polygon', name, codeDrawOptions, userDrawOptions);
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

class Area extends Polygon
{
    constructor(codeDrawOptions={}, userDrawOptions={}) {
        super('Area', codeDrawOptions, userDrawOptions);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}

