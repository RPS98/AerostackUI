class Polygon extends DrawManager
{
    constructor(name, options = undefined) {
        super('Polygon', name, options);
    }
}

class Area extends Polygon
{
    constructor(options = undefined) {
        super('Area', options);
    }
}

