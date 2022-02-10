class Line extends DrawManager
{
    constructor(name, codeDrawOptions={}, userDrawOptions={}) {
        super('Line', name, codeDrawOptions, userDrawOptions);
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

class Path extends Line
{
    constructor(codeDrawOptions={}, userDrawOptions={}) {
        super('Path', codeDrawOptions, userDrawOptions);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}