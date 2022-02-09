class DrawManager
{
    constructor (type, codeDrawOptions={}, userDrawOptions={}) {
        this.type = type
        this.codeDrawOptions = Object.assign(
            {}, 
            codeDrawOptions, 
            {
                'author': 'manager',
                'type': type,
                'instance': this 
            }
        );
        this.userDrawOptions = Object.assign(
            {}, 
            userDrawOptions, 
            {
                'author': 'user',
                'status': 'draw',
                'type': type,
                'instance': this 
            }
        );
    }

    codeDraw(values, options={}) {

        let draw = null;

        switch (this.type) {
            case 'Marker':
                draw = L.marker(values, Object.assign({}, this.codeDrawOptions, options));
                break;
            case 'Line':
                draw = L.polyline(values, Object.assign({}, this.codeDrawOptions, options));
                break;
            case 'Circle':
                draw = L.circle(values[0], values[1], Object.assign({}, this.codeDrawOptions, options));
                break;
            case 'Polygon':
                draw = L.polygon(values, Object.assign({}, this.codeDrawOptions, options));
                break;
            default:
                alert("Try to draw from code a type: " + this.type);
                throw new Error("Unknown type of draw");
        }
        draw.addTo(MAP);
        return draw
    }

    userDraw(options={}) {

        switch (this.type) {
            case 'Marker':
                MAP.pm.enableDraw('Marker', Object.assign({}, this.userDrawOptions, options));
                break;
            case 'Line':
                MAP.pm.enableDraw('Line', Object.assign({}, this.userDrawOptions, options));
                break;
            case 'Circle':
                MAP.pm.enableDraw('Circle', Object.assign({}, this.userDrawOptions, options));
                break;
            case 'Polygon':
                MAP.pm.enableDraw('Polygon', Object.assign({}, this.userDrawOptions, options));
                break;
            case 'CircleMarker':
                MAP.pm.enableDraw('CircleMarker', Object.assign({}, this.userDrawOptions, options));
                break;
            case 'Rectangle':
                MAP.pm.enableDraw('Rectangle', Object.assign({}, this.userDrawOptions, options));
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