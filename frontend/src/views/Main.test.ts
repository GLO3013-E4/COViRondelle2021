import Main from '@/views/Main.vue';
import wrapWithVuetify from '@/util/wrapWithVuetify';

const wrapper = wrapWithVuetify(Main);

describe('When mounting main view', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
