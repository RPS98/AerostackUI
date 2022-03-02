var ID = 0;

class DrawManager {
    constructor(type, name, options = {}) {
        this.type = type
        this.options = Object.assign(
            {
                'instance': this,
                'type': type,
                'name': name,
            },
            options,
        );

        this.codeLayerDrawn = null;
    }

    codeDraw(values, options = {}) {
        let drawManagerOptions = Object.assign({}, this.options, {'drawUserOptions': options, 'id': ID++});
        let drawOptions = Object.assign({'DrawManager': drawManagerOptions}, options);

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

    userDraw(options = {}) {
        options['author'] = 'user';
        options['status'] = 'draw';
        let drawManagerOptions = Object.assign({}, this.options, {'drawCodeOptions': options, 'id': ID++});
        let drawOptions = Object.assign({'DrawManager': drawManagerOptions}, options, this.options);

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

    getHtmlDrawInfo(options) {
        //throw new Error("Method not implemented.");
        console.log("Method not implemented.");
    }

    getHtmlCodeInfo() {
        throw new Error("Method not implemented.");
    }

    getLayerInfo() {
        throw new Error("Method not implemented.");
    }
}