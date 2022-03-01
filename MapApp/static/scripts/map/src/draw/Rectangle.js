class Rectangle extends DrawManager
{
    constructor(name, globalOptions = undefined, codeDrawOptions = undefined, userDrawOptions = undefined) {
        super('Rectangle', name, globalOptions, codeDrawOptions, userDrawOptions);
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
