class Polygon extends DrawManager
{
    constructor(name, codeDrawOptions={}, userDrawOptions={}) {
        super('Polygon', name, codeDrawOptions, userDrawOptions);
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

