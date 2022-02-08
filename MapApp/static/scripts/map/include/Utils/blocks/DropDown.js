class DropDown extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'id': id,
                'text': args[0],
                'btn': args[1],
                'expand': args[2],
            }
        );
    }  

    static addHTML(content) {

        let div = document.createElement('div');
        div.setAttribute('class', 'row');

        content.btn.attributes['class'] += ' dropdown-toggle';
        content.btn.attributes['data-bs-toggle'] = 'dropdown';
        content.btn.attributes['aria-expanded'] = 'false';
        HTMLUtils.addHTML(div, content.btn);

        content.expand.attributes['class'] += ' dropdown-menu';
        HTMLUtils.addHTML(div, content.expand);

        return div;
    }
}