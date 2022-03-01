class Circle extends DrawManager
{
    constructor(name, options = undefined) {
        super('Circle', name, options);
    }
}

class CircularArea extends Circle
{
    constructor(options = undefined) {
        super('CircularArea', options);
    }
}
