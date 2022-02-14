class Circle extends DrawManager
{
    constructor(name, codeDrawOptions={}, userDrawOptions={}) {
        super('Circle', name, codeDrawOptions, userDrawOptions);
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
    constructor(codeDrawOptions={}, userDrawOptions={}) {
        super('CircularArea', codeDrawOptions, userDrawOptions);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}
