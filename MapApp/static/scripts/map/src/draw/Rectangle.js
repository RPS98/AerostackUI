class Rectangle extends DrawManager
{
    constructor(name, options = undefined) {
        super('Rectangle', name, options);
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
