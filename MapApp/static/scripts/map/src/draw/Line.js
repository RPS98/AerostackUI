class Line extends DrawManager
{
    constructor(name, codeDrawOptions={}, userDrawOptions={}) {
        super('Line', name, codeDrawOptions, userDrawOptions);
    }

    codeDraw(values, options={}) {
        if (values.length > 1)
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

class Path extends Line
{
    constructor(codeDrawOptions={}, userDrawOptions={}) {
        super('path', codeDrawOptions, userDrawOptions);
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
    constructor(codeDrawOptions={}, userDrawOptions={}) {
        super('odom', codeDrawOptions, userDrawOptions);
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
    constructor(codeDrawOptions={}, userDrawOptions={}) {
        super('desiredPath', codeDrawOptions, userDrawOptions);
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