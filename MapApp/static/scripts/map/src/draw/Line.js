class Line extends DrawManager
{
    constructor(name, options = undefined) {
        super('Line', name, options);
    }

    codeDraw(values, options={}) {
        if (values.length > 1) {
            return super.codeDraw(values, options);
        }
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

class Path extends Line
{
    constructor(options = undefined) {
        super('Path', options);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}


class Odom extends Line
{
    constructor(options = undefined) {
        super('Odom', options);
    }

    codeDraw(id, values, options={}) {
        options['color'] = M.UAV_MANAGER.getColors(id)[1];
        return super.codeDraw(values, options);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}


class DesiredPath extends Line
{
    constructor(options = undefined) {
        super('DesiredPath', options);
    }

    codeDraw(id, values, options={}) {
        options['color'] = M.UAV_MANAGER.getColors(id)[1];
        options['opacity'] = 0.5;

        return super.codeDraw(values, options);
    }

    showInfo() {
        super.showInfo();
    }

    sendInfo() {
        super.sendInfo();
    }
}