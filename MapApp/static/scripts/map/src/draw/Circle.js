class Circle extends DrawManager
{
    constructor(name, globalOptions = undefined, codeDrawOptions = undefined, userDrawOptions = undefined) {
        super('Circle', name, globalOptions, codeDrawOptions, userDrawOptions);
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

class CircularArea extends Circle
{
    constructor(globalOptions = undefined, codeDrawOptions = undefined, userDrawOptions = undefined) {
        super('CircularArea', globalOptions, codeDrawOptions, userDrawOptions);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}
