import StationInformation from '@/components/station/StationInformation.vue';
import wrapWithVuetify from '@/util/wrapWithVuetify';

const wrapper = wrapWithVuetify(StationInformation);

describe('When mounting station information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
