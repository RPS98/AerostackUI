class Polygon extends DrawManager
{
    constructor(name, globalOptions = undefined, codeDrawOptions = undefined, userDrawOptions = undefined) {
        super('Polygon', name, globalOptions, codeDrawOptions, userDrawOptions);
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
    constructor(globalOptions = undefined, codeDrawOptions = undefined, userDrawOptions = undefined) {
        super('Area', globalOptions, codeDrawOptions, userDrawOptions);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}

