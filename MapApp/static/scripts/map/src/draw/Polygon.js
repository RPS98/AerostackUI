class Polygon extends DrawManager
{
    constructor(name, options = undefined) {
        super('Polygon', name, options);
    }

    _addChangeCallback(id, info) {
        let htmlId = [];
        let nameId = [];
        let values = info.layer._latlngs[0];
        for (let i = 0; i < values.length; i++) {
            htmlId.push(`${id}-${i}-lat`);
            htmlId.push(`${id}-${i}-lng`);

            nameId.push(`${i}-lat`);
            nameId.push(`${i}-lng`);
        }
        Utils.addFormCallback(`${id}-change`, htmlId, nameId, this._changeCallback.bind(this), id, info);
    }

    _changeCallback(myargs, inputs) {
        let layer = myargs[1].layer;
        
        let values = [];
        let len = Math.floor(Object.keys(inputs).length / 2);
        for (let i = 0; i < len; i++) {
            values.push([]);
        }
        for (var key in inputs) {
            let value = inputs[key];
            let keyParse = key.split('-');
            let index1 = parseInt(keyParse[0]);
            if (keyParse[1] == 'lat') {
                values[index1][0] = value;
            } else if (keyParse[1] == 'lng') {
                values[index1][1] = value;
            }
        }

        layer.setLatLngs(values);
    }

    drawInfoAdd(htmlId, info, name = "Line", initialHtml = [], endHtml = undefined, uavPickerType = undefined) {

        let id = htmlId + '-' + info.id;

        let values = info.layer._latlngs[0];
        for (let i = 0; i < values.length; i++) {
            let lat = values[i].lat;
            let lng = values[i].lng;

            let latDict = HTMLUtils.addDict('input', `${id}-${i}-lat`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lat, 6) }, 'number', 'Latitude');
            let lngDict = HTMLUtils.addDict('input', `${id}-${i}-lng`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lng, 6) }, 'number', 'Longitude');
            let row = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [latDict, lngDict], { 'class': 'col-6' });

            initialHtml.push(row);
        }

        return super.drawInfoAdd(htmlId, info, name, initialHtml, endHtml, uavPickerType);
    }
}

class Area extends Polygon
{
    constructor(options = undefined) {
        super('Area', options);
    }

    codeDraw(id, values, options = {}) {
        options['color'] = M.MISSION_MANAGER.getColors(id)[1];
        return super.codeDraw(values, options);
    }

    drawInfoAdd(htmlId, info) {
        let name = 'Area';
        let endHtml = [];
        let algorithms = ['Back and force', 'Spiral'];
        let id = htmlId + '-' + info.id;
        endHtml.push(HTMLUtils.initDropDown(`${id }-Swarming`, algorithms, 'Back and force'));

        let streetSpacing = HTMLUtils.addDict('input', `${id}-streetSpacing`, { 'class': 'form-control', 'required': 'required', 'value': 1}, 'number', 'Value');
        let streetSpacingBtn = HTMLUtils.addDict('button', `${id}-streetSpacingBtn`, { 'class': 'btn btn-primary' }, 'Set street space (m)');
        let streetSpacingDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [streetSpacing]);
        let streetSpacingBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-6' }, [streetSpacingBtn]);
        endHtml.push(HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [streetSpacingDiv, streetSpacingBtnDiv]));

        let wpSpace = HTMLUtils.addDict('input', `${id}-wpSpace`, { 'class': 'form-control', 'required': 'required', 'value': 1}, 'number', 'Value');
        let wpSpaceBtn = HTMLUtils.addDict('button', `${id}-wpSpaceBtn`, { 'class': 'btn btn-primary' }, 'Set wp space (m)');
        let wpSpaceDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [wpSpace]);
        let wpSpaceBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-6' }, [wpSpaceBtn]);
        endHtml.push(HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [wpSpaceDiv, wpSpaceBtnDiv]));

        return super.drawInfoAdd(htmlId, info, name, undefined, endHtml, 'checkbox');
    }

    drawInfoInitialize(id, info) {
        info.drawManager.drawUserOptions.algorithm = 'Back and force';
        Utils.addButtonsCallback(`${id}-Swarming-item`, this.clickAlgorithmsListCallback.bind(this), id, info);

        info.drawManager.drawUserOptions.streetSpacing = 1;
        Utils.addFormCallback(`${id}-streetSpacingBtn`, [`${id}-streetSpacing`], ['streetSpacingValue'], this.streetSpacingCallback.bind(this), id, info);

        info.drawManager.drawUserOptions.wpSpace = 1;
        Utils.addFormCallback(`${id}-wpSpaceBtn`, [`${id}-wpSpace`], ['wpSpaceValue'], this.wpSpaceCallback.bind(this), id, info);

        super.drawInfoInitialize(id, info);
    }

    clickAlgorithmsListCallback(e, args) {
        args[1].drawManager.drawUserOptions.algorithm = e.innerText;

        let button = document.getElementById(`${args[0]}-Swarming-DropDown-Btn`);
        button.innerHTML = e.innerHTML;
    }

    streetSpacingCallback(myargs, inputs) {
        myargs[1].drawManager.drawUserOptions.streetSpacing = inputs.streetSpacingValue;
    }

    wpSpaceCallback(myargs, inputs) {
        myargs[1].drawManager.drawUserOptions.wpSpace = inputs.wpSpaceValue;
    }
}

