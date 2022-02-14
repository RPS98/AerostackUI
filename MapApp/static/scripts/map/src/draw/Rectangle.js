class Rectangle extends DrawManager
{
    constructor(name, codeDrawOptions={}, userDrawOptions={}) {
        super('Rectangle', name, codeDrawOptions, userDrawOptions);
    }

    codeDraw(values, options={}) {
        // TODO: Make rectangle draw on MAP
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
