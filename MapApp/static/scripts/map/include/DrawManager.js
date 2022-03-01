class DrawManager
{
    constructor (type, name, globalOptions={}, codeDrawOptions={}, userDrawOptions={}) {
        this.type = type
        this.globalOptions = Object.assign(
            {
                'DrawManager': this,
            },
            globalOptions,
        );


        this.codeDrawOptions = Object.assign(
            this.globalOptions, 
            {
                'author': 'manager',
                'type': type,
                'name': name,
            },
            codeDrawOptions
        );
        this.userDrawOptions = Object.assign(
            this.globalOptions, 
            {
                'author': 'user',
                'type': type,
                'name': name,
                'status': 'draw',
            },
            userDrawOptions
        );
        this.codeLayerDrawn = null;
    }

    codeDraw(values, options={}) {
        let drawOptions = Object.assign({}, this.codeDrawOptions, options);

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
        draw.addTo(M.MAP);
        this.codeLayerDrawn = draw;
        return draw
    }

    userDraw(options={}) {
        let drawOptions = Object.assign({}, this.userDrawOptions, options);

        switch (this.type) {
            case 'Marker':
                M.MAP.pm.enableDraw('Marker', drawOptions);
                break;
            case 'Line':
                M.MAP.pm.enableDraw('Line', drawOptions);
                break;
            case 'Circle':
                M.MAP.pm.enableDraw('Circle', drawOptions);
                break;
            case 'Polygon':
                M.MAP.pm.enableDraw('Polygon', drawOptions);
                break;
            case 'CircleMarker':
                M.MAP.pm.enableDraw('CircleMarker', drawOptions);
                break;
            case 'Rectangle':
                M.MAP.pm.enableDraw('Rectangle', drawOptions);
                break;
            default:
                alert("Try to draw from code a type: " + this.type);
                throw new Error("Unknown type of draw");
        }
    }

    showDrawInfo() {
        throw new Error("Method not implemented.");
    }

    showCodeInfo() {
        throw new Error("Method not implemented.");
    }

    getLayerInfo() {
        throw new Error("Method not implemented.");
    }
}