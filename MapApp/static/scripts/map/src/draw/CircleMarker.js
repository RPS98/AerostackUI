class CircleMarker extends DrawManager
{
    constructor(name, codeDrawOptions={}, userDrawOptions={}) {
        super('CircleMarker', name, codeDrawOptions, userDrawOptions);
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
