class ImageOVerlay {
    constructor() {
        this.htmlId = 'sideBar-left-imageOverlay-content';

        this.point_list = [];
        this.point_list.push(L.latLng(28.14422, -16.50366));
        this.point_list.push(L.latLng(28.14418, -16.50050));
        this.point_list.push(L.latLng(28.14287, -16.50366));

        // Drag icon image
        let imageOverlayURL = "/static/scripts/map/img/arrows-alt-solid.svg";

        // Set image draggable icons
        let imageOverlay = L.icon({ iconUrl: imageOverlayURL, iconSize: [30, 30] });

        this.markerTL = L.marker(this.point_list[0], { draggable: true, pmIgnore: true, icon: imageOverlay })
        this.markerTR = L.marker(this.point_list[1], { draggable: true, pmIgnore: true, icon: imageOverlay })
        this.markerBL = L.marker(this.point_list[2], { draggable: true, pmIgnore: true, icon: imageOverlay })

        this.overlay = null;
        this.imgUrl = "/static/scripts/map/img/Global1.JPG";
        this.opacity = 0.7;

        this.addHTML();
        this.addCallbacks();
    }

    addHTML() {
        let htmlList = [];

        // let file = HTMLUtils.addDict('fileInput', `${this.htmlId}-imageFile`, {}, 'Choose Image', this.imgUrl);

        let fileInput = HTMLUtils.addDict('input', `${this.htmlId}-imageFilenput`, { 'class': 'form-control', 'required': 'required' }, 'url', `Image URL`);
        let fileBtn = HTMLUtils.addDict('button', `${this.htmlId}-imageFileBtn`, { 'class': 'btn btn-primary' }, 'Add URL');
        let fileRow = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [fileInput, fileBtn], { 'class': 'col-md-6' });

        let topLeftLat = HTMLUtils.addDict('input', `${this.htmlId}-topLeftLat`, { 'class': 'form-control', 'required': 'required', 'value': this.point_list[0].lat }, 'text', 'Latitude');
        let topLeftLng = HTMLUtils.addDict('input', `${this.htmlId}-topLeftLng`, { 'class': 'form-control', 'required': 'required', 'value': this.point_list[0].lng }, 'text', 'Latitude');
        let row1 = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [topLeftLat, topLeftLng], { 'class': 'col-6' });

        let toprightLat = HTMLUtils.addDict('input', `${this.htmlId}-toprightLat`, { 'class': 'form-control', 'required': 'required', 'value': this.point_list[1].lat }, 'text', 'Latitude');
        let toprightLng = HTMLUtils.addDict('input', `${this.htmlId}-toprightLng`, { 'class': 'form-control', 'required': 'required', 'value': this.point_list[1].lng }, 'text', 'Latitude');
        let row2 = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [toprightLat, toprightLng], { 'class': 'col-6' });

        let bottomLeftLat = HTMLUtils.addDict('input', `${this.htmlId}-bottomLeftLat`, { 'class': 'form-control', 'required': 'required', 'value': this.point_list[2].lat }, 'text', 'Latitude');
        let bottomLeftLng = HTMLUtils.addDict('input', `${this.htmlId}-bottomLeftLng`, { 'class': 'form-control', 'required': 'required', 'value': this.point_list[2].lng }, 'text', 'Latitude');
        let row3 = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [bottomLeftLat, bottomLeftLng], { 'class': 'col-6' });

        let addBtn = HTMLUtils.addDict('button', `${this.htmlId}-addOverlayBtn`, { 'class': 'btn btn-primary' }, 'Add Image');
        let removeBtn = HTMLUtils.addDict('button', `${this.htmlId}-removeOverlayBtn`, { 'class': 'btn btn-danger' }, 'Remove Image');
        let addBtnDiv = HTMLUtils.addDict('div', `none`, {}, [addBtn]);
        let removeBtniv = HTMLUtils.addDict('div', `none`, {}, [removeBtn]);
        let addRemoveBtn = HTMLUtils.addDict('div', `none`, { 'class': 'btn-group d-flex justify-content-evenly', 'role': 'group' }, [addBtnDiv, removeBtniv]);

        let opacityInput = HTMLUtils.addDict('input', `${this.htmlId}-opacityInput`, { 'class': 'form-control', 'required': 'required',  'value': this.opacity }, 'text', `Opacity`);
        let opacityBtn = HTMLUtils.addDict('button', `${this.htmlId}-opacityBtn`, { 'class': 'btn btn-primary' }, 'Set opacity (m)');
        let opacityRow = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [opacityInput, opacityBtn], { 'class': 'col-md-6' });

        // htmlList.push(file);
        htmlList.push(fileRow);
        htmlList.push(row1);
        htmlList.push(row2);
        htmlList.push(row3);
        htmlList.push(addRemoveBtn);
        htmlList.push(opacityRow);

        HTMLUtils.addToExistingElement(`${this.htmlId}`, htmlList);
    }

    addCallbacks() {

        // Utils.addFileCallback(`${this.htmlId}-imageFile`, this.addImageUrlCallback.bind(this));
        Utils.addFormCallback(`${this.htmlId}-imageFileBtn`, [`${this.htmlId}-imageFilenput`], ['imgURL'], this.addImageUrlCallback.bind(this));

        Utils.addFormCallback(`${this.htmlId}-opacityBtn`, [`${this.htmlId}-opacityInput`], ['opacity'], this.setOpacity.bind(this));
        Utils.addButtonCallback(`${this.htmlId}-removeOverlayBtn`, this._removeCallback.bind(this));

        Utils.addFormCallback(
            `${this.htmlId}-addOverlayBtn`,
            [
                `${this.htmlId}-topLeftLat`,    `${this.htmlId}-topLeftLng`,
                `${this.htmlId}-toprightLat`,   `${this.htmlId}-toprightLng`,
                `${this.htmlId}-bottomLeftLat`, `${this.htmlId}-bottomLeftLng`
            ],
            [
                `topLeftLat`,    `topLeftLng`,
                `toprightLat`,   `toprightLng`,
                `bottomLeftLat`, `bottomLeftLng`
            ],
            this.addImageOverlayCallback.bind(this)
        );

        
    }

    addImageUrlCallback(args, inputs) {
        console.log("addImageUrlCallback");
        this.imgUrl = inputs.imgURL;
    }

    addImageOverlayCallback(args, inputs) {

        if (this.imgUrl == null) {
            alert("Please select an image");
            return;
        }

        // Create three markers to drag the image
        this.markerTL.addTo(M.MAP);
        this.markerTR.addTo(M.MAP);
        this.markerBL.addTo(M.MAP);

        this.markerTL.setLatLng(L.latLng(inputs['topLeftLat'], inputs['topLeftLng']));
        this.markerTR.setLatLng(L.latLng(inputs['toprightLat'], inputs['toprightLng']));
        this.markerBL.setLatLng(L.latLng(inputs['bottomLeftLat'], inputs['bottomLeftLng']));

        // Create the image overlay
        console.log("Adding image");
        console.log(this.imgUrl);
        if (this.overlay == null) {
            this.overlay = L.imageOverlay.rotated(this.imgUrl, this.point_list[0], this.point_list[1], this.point_list[2], {
                opacity: this.opacity,
                interactive: true,
            });
        } else {
            this.overlay.setUrl(this.imgUrl);
            this.overlay.setBounds(this.point_list);
            this.overlay.setOpacity(this.opacity);
        }

        this.markerTL.on('drag dragend', this.repositionImage.bind(this));
        this.markerTR.on('drag dragend', this.repositionImage.bind(this));
        this.markerBL.on('drag dragend', this.repositionImage.bind(this));

        // Add the image overlay to the map
        M.MAP.addLayer(this.overlay, { pmIgnore: true });
    }

    _removeCallback() {
        if (this.overlay != null) {
            M.MAP.removeLayer(this.overlay);
            this.markerTL.remove();
            this.markerTR.remove();
            this.markerBL.remove();
            this.overlay = null;
        }
    }

    repositionImage() {
        this.overlay.reposition(
            this.markerTL.getLatLng(), 
            this.markerTR.getLatLng(), 
            this.markerBL.getLatLng()
        );
    }

    setOpacity(args, inputs) {

        this.opacity = inputs['opacity'];
        if (this.overlay != null) {
            this.overlay.setOpacity(this.opacity);
        }
    }

}