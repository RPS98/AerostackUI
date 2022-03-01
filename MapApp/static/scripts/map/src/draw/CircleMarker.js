class CircleMarker extends DrawManager
{
    constructor(name, globalOptions = undefined, codeDrawOptions = undefined, userDrawOptions = undefined) {
        super('CircleMarker', name, globalOptions, codeDrawOptions, userDrawOptions);
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
