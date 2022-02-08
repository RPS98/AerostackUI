class Button extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'text': args[0],
            }
        );
    }   

    static addHTML(content) {
        let button = document.createElement('button');
        button.setAttribute('type', 'button');
        button.innerHTML = content.text;
        return button;
    }
}