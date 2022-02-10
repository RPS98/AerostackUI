class Rectangle extends DrawManager
{
    constructor(name, codeDrawOptions={}, userDrawOptions={}) {
        super('Rectangle', name, codeDrawOptions, userDrawOptions);
    }

    codeDraw(layer, values, options={}) {
        // TODO: Make rectangle draw on MAP
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
