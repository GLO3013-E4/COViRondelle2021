import Main from '@/views/Main.vue';
import useVuetify from '@/hooks/useVuetify';

const wrapper = useVuetify(Main);

describe('When mounting main view', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
