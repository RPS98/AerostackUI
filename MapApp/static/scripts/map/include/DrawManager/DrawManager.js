class DrawManager
{
    constructor (type, name, codeDrawOptions={}, userDrawOptions={}) {
        this.type = type
        this.codeDrawOptions = Object.assign(
            {}, 
            {
                'author': 'manager',
                'type': type,
                'name': name,
                'instance': this 
            },
            codeDrawOptions
        );
        this.userDrawOptions = Object.assign(
            {}, 
            {
                'author': 'user',
                'type': type,
                'name': name,
                'status': 'draw',
                'instance': this 
            },
            userDrawOptions
        );
    }

    codeDraw(layer, values, options={}) {
        let layerOptions = Object.assign({}, layer.getInfo(), options);
        let drawOptions = Object.assign({}, this.codeDrawOptions, layerOptions);

        let draw = null;

        switch (this.type) {
            case 'Marker':
                draw = L.marker(values, drawOptions);
                break;
            case 'Line':
                draw = L.polyline(values, drawOptions);
                break;
            case 'Circle':
                draw = L.circle(values[0], values[1], drawOptions);
                break;
            case 'Polygon':
                draw = L.polygon(values, drawOptions);
                break;
            default:
                alert("Try to draw from code a type: " + this.type);
                throw new Error("Unknown type of draw");
        }
        draw.addTo(MAP_MANAGER.MAP);
        return draw
    }

    userDraw(layer, options={}) {
        let layerOptions = Object.assign({}, layer.getInfo(), options);
        let drawOptions = Object.assign({}, this.userDrawOptions, layerOptions);

        switch (this.type) {
            case 'Marker':
                MAP_MANAGER.MAP.pm.enableDraw('Marker', drawOptions);
                break;
            case 'Line':
                MAP_MANAGER.MAP.pm.enableDraw('Line', drawOptions);
                break;
            case 'Circle':
                MAP_MANAGER.MAP.pm.enableDraw('Circle', drawOptions);
                break;
            case 'Polygon':
                MAP_MANAGER.MAP.pm.enableDraw('Polygon', drawOptions);
                break;
            case 'CircleMarker':
                MAP_MANAGER.MAP.pm.enableDraw('CircleMarker', drawOptions);
                break;
            case 'Rectangle':
                MAP_MANAGER.MAP.pm.enableDraw('Rectangle', drawOptions);
                break;
            default:
                alert("Try to draw from code a type: " + this.type);
                throw new Error("Unknown type of draw");
        }
    }

    showInfo() {
        throw new Error("Method not implemented.");
    }

    sendInfo() {
        throw new Error("Method not implemented.");
    }
}