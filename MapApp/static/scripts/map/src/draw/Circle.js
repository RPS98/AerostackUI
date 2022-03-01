class Circle extends DrawManager
{
    constructor(name, options = undefined) {
        super('Circle', name, options);
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
    constructor(options = undefined) {
        super('CircularArea', options);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}
